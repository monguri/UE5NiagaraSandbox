#include "RopeSimulatorCPU.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Texture2D.h"
#include "Components/ArrowComponent.h"
#include "Components/BillboardComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/SphereComponent.h"
#include "Components/CapsuleComponent.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/Character.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Async/ParallelFor.h"

void ARopeSimulatorCPU::PreInitializeComponents()
{
	Super::PreInitializeComponents();

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();

	Positions.SetNum(NumRopes);
	PrevPositions.SetNum(NumRopes);
	PrevConstraintSolvePositions.SetNum(NumRopes);
	Velocities.SetNum(NumRopes);
	PrevConstraintSolveVelocities.SetNum(NumRopes);
	PrevSolveVelocities.SetNum(NumRopes);
	Colors.SetNum(NumRopes);
	Accelerations.SetNum(NumRopes);

	for (int32 ParticleIdx = 0; ParticleIdx < NumRopes; ++ParticleIdx)
	{
		PrevSolveVelocities[ParticleIdx] = PrevConstraintSolveVelocities[ParticleIdx] = Velocities[ParticleIdx] = FVector::ZeroVector;
		Accelerations[ParticleIdx] = InvActorTransform.TransformVectorNoScale(FVector(0.0f, 0.0f, Gravity));

		Colors[ParticleIdx] = FLinearColor::MakeRandomColor();

		// スリープの仕組みを使うケースでは初期化時は全パーティクルスリープ状態から開始する
	}

	// Tick()で設定しても、レベルにNiagaraSystemが最初から配置されていると、初回のスポーンでは配列は初期値を使ってしまい
	//間に合わないのでBeginPlay()でも設定する
	NiagaraComponent->SetNiagaraVariableInt("NumParticles", NumRopes);
	// メッシュはシリンダーに固定
	NiagaraComponent->SetNiagaraVariableInt("Shape", 1);
	NiagaraComponent->SetNiagaraVariableVec3("MeshScale", FVector(MeshScale, MeshScale, MeshScale * 0.2f) * 2); // 使うメッシュが1x1x1で半径0.5なので2倍にしておく。厚みは幅の0.2倍に
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions);
	// TODO:
	//UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayQuat(NiagaraComponent, FName("Orientations"), Orientations);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("Colors"), Colors);

	NumThreadParticles = (NumRopes + NumThreads - 1) / NumThreads;

	for (const AActor* CollisionActor : CollisionActors)
	{
		// CollisionActorsはプレー中に動的に変化しない前提
		const UPrimitiveComponent* Primitive = CollisionActor->FindComponentByClass<UPrimitiveComponent>();
		if (Primitive == nullptr || Primitive->GetBodyInstance() == nullptr || Primitive->GetBodyInstance()->GetBodySetup() == nullptr)
		{
			// CollisionActors内でBodySetupをもたないアクタはプレー中は変化しないという前提
			continue;
		}

		// とりあえずワールド座標の状態で保存するがすぐUpdateActorCollisions()でローカル座標に変換する
		ActorsAggGeom.Add(Primitive->GetBodyInstance()->GetBodySetup()->AggGeom);
	}

	// プレーを開始した最初のフレームでActorsAggGeomとPrevActorsAggGeomを初期ポーズで初期化しておく
	UpdateActorCollisions();
	PrevActorsAggGeom = ActorsAggGeom;
}

void ARopeSimulatorCPU::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		// DeltaSecondsの値の変動に関わらず、シミュレーションに使うサブステップタイムは固定とする
		float FixedDeltaSeconds = 1.0f / FrameRate;
		float SubStepDeltaSeconds = FixedDeltaSeconds / NumSubStep;

		UpdateActorCollisions();
		UpdateCoinBlockers();

		// Niagara版はParticleAttributeReaderの値がSimStageのイテレーション単位でしか更新されない。
		// そのため、サブステップを複数にしても、Integrate()で落ちた位置をそのサブステップ回でParticleAttributeReaderから参照できない。
		// となると形状維持コンストレイントのために自由落下が妨げられて遅くなる。
		for (int32 SubStep = 0; SubStep < NumSubStep; ++SubStep)
		{
			// マルチスレッド化するほどの処理でもないのでしない
			for (int32 ParticleIdx = 0; ParticleIdx < NumRopes; ++ParticleIdx)
			{
				Accelerations[ParticleIdx] = InvActorTransform.TransformVectorNoScale(FVector(0.0f, 0.0f, Gravity));

				Integrate(ParticleIdx, SubStepDeltaSeconds);
			}


			for (int32 Iter = 0; Iter < NumIteration; ++Iter)
			{
				SolvePositionConstraint(SubStep * NumIteration + Iter); 
			}

			SolveVelocity(DeltaSeconds, SubStepDeltaSeconds, SubStep);
		}
	}

	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions);
	//TODO:
	//UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayQuat(NiagaraComponent, FName("Orientations"), Orientations);
}

bool ARopeSimulatorCPU::IsCollisioned(const FVector& Position) const
{
	for (const AActor* CollisionActor : CollisionActors)
	{
		const FBox& WorldBoxBound = CollisionActor->GetComponentsBoundingBox(false, false);
		const FBox& BoxBound = WorldBoxBound.TransformBy(InvActorTransform);
		// Boxと点のコリジョン判定にするために半径分だけBoxサイズを拡張する
		const FBox& ExpandedBoxBound = BoxBound.ExpandBy(RopeRadius);
		if (ExpandedBoxBound.IsInsideOrOn(Position))
		{
			return true;
		}
	}

	for (TPair<TWeakObjectPtr<class UPrimitiveComponent>, FTransform> CollisionPose : CoinBlockerCollisionsPoseMap)
	{
		if (CollisionPose.Key == nullptr)
		{
			continue;
		}

		const FBox& WorldBoxBound = CollisionPose.Key->Bounds.GetBox();
		const FBox& BoxBound = WorldBoxBound.TransformBy(InvActorTransform);
		// Boxと点のコリジョン判定にするために半径分だけBoxサイズを拡張する
		const FBox& ExpandedBoxBound = BoxBound.ExpandBy(RopeRadius);
		if (ExpandedBoxBound.IsInsideOrOn(Position))
		{
			return true;
		}
	}

	return false;
}

void ARopeSimulatorCPU::UpdateActorCollisions()
{
	PrevActorsAggGeom = ActorsAggGeom;

	int32 Counter = 0;
	for (const AActor* CollisionActor : CollisionActors)
	{
		// CollisionActorsはプレー中に動的に変化しない前提
		const UPrimitiveComponent* Primitive = CollisionActor->FindComponentByClass<UPrimitiveComponent>();
		if (Primitive == nullptr || Primitive->GetBodyInstance() == nullptr || Primitive->GetBodyInstance()->GetBodySetup() == nullptr)
		{
			// CollisionActors内でBodySetupをもたないアクタはプレー中は変化しないという前提
			continue;
		}

		// ローカル座標に変換
		const FTransform& ActorTM = CollisionActor->GetTransform() * InvActorTransform;

		const FKAggregateGeom& OriginalAggGeom = Primitive->GetBodyInstance()->GetBodySetup()->AggGeom;
		FKAggregateGeom& CacheAggGeom = ActorsAggGeom[Counter];
		Counter++;

		for (int32 i = 0; i < OriginalAggGeom.SphereElems.Num(); ++i)
		{
			// CollisionGeometryVisualization.cppのUE::PhysicsTools::InitializePreviewGeometryLines()を参考にしている
			CacheAggGeom.SphereElems[i] = OriginalAggGeom.SphereElems[i].GetFinalScaled(FVector::OneVector, ActorTM);
			CacheAggGeom.SphereElems[i].Radius = OriginalAggGeom.SphereElems[i].Radius * ActorTM.GetScale3D().GetAbsMax(); 
		}

		for (int32 i = 0; i < OriginalAggGeom.BoxElems.Num(); ++i)
		{
			// CollisionGeometryVisualization.cppのUE::PhysicsTools::InitializePreviewGeometryLines()を参考にしている
			CacheAggGeom.BoxElems[i] = OriginalAggGeom.BoxElems[i].GetFinalScaled(FVector::OneVector, ActorTM);
			CacheAggGeom.BoxElems[i].X = OriginalAggGeom.BoxElems[i].X * ActorTM.GetScale3D().X;
			CacheAggGeom.BoxElems[i].Y = OriginalAggGeom.BoxElems[i].Y * ActorTM.GetScale3D().Y;
			CacheAggGeom.BoxElems[i].Z = OriginalAggGeom.BoxElems[i].Z * ActorTM.GetScale3D().Z;
		}

		for (int32 i = 0; i < OriginalAggGeom.SphylElems.Num(); ++i)
		{
			// CollisionGeometryVisualization.cppのUE::PhysicsTools::InitializePreviewGeometryLines()を参考にしている
			CacheAggGeom.SphylElems[i] = OriginalAggGeom.SphylElems[i].GetFinalScaled(FVector::OneVector, ActorTM);
			CacheAggGeom.SphylElems[i].Radius = OriginalAggGeom.SphylElems[i].GetScaledRadius(ActorTM.GetScale3D());
			CacheAggGeom.SphylElems[i].Length = OriginalAggGeom.SphylElems[i].GetScaledCylinderLength(ActorTM.GetScale3D());
		}
	}
}

void ARopeSimulatorCPU::UpdateCoinBlockers()
{
	TArray<FOverlapResult> Overlaps;
	FCollisionObjectQueryParams ObjectParams(OverlapQueryObjectTypes);
	if (!ObjectParams.IsValid())
	{
		return;
	}

	FCollisionQueryParams Params(SCENE_QUERY_STAT(UpdateCoinBlockers), false);

	GetWorld()->OverlapMultiByObjectType(Overlaps, GetActorLocation() + WallBox.GetCenter(), GetActorQuat(), ObjectParams, FCollisionShape::MakeBox(WallBox.GetExtent()), Params);

	// キャラクターが増減したりコリジョンコンポーネントが増減することを想定し、呼ばれるたびに配列を作り直す
	PrevCoinBlockerCollisionsPoseMap = CoinBlockerCollisionsPoseMap;
	CoinBlockerCollisionsPoseMap.Reset();

	for (const FOverlapResult& Overlap : Overlaps)
	{
		if (Overlap.Component != nullptr)
		{
			CoinBlockerCollisionsPoseMap.Add(Overlap.Component, Overlap.Component->GetComponentTransform() * InvActorTransform);
		}
	}
}

void ARopeSimulatorCPU::Integrate(int32 ParticleIdx, float SubStepDeltaSeconds)
{
	PrevPositions[ParticleIdx] = Positions[ParticleIdx];
	Velocities[ParticleIdx] += Accelerations[ParticleIdx] * SubStepDeltaSeconds;
	Positions[ParticleIdx] += Velocities[ParticleIdx] * SubStepDeltaSeconds;
	PrevConstraintSolvePositions[ParticleIdx] = Positions[ParticleIdx];
}

void ARopeSimulatorCPU::SolvePositionConstraint(int32 InFrameExeCount)
{
	ParallelFor(NumThreads,
		[this, InFrameExeCount](int32 ThreadIndex)
		{
			for (int32 ParticleIdx = NumThreadParticles * ThreadIndex; ParticleIdx < NumThreadParticles * (ThreadIndex + 1) && ParticleIdx < NumRopes; ++ParticleIdx)
			{
				ApplyCollisionActorsConstraint(ParticleIdx, InFrameExeCount);
				ApplyCoinBlockersCollisionConstraint(ParticleIdx, InFrameExeCount);
				ApplyWallCollisionConstraint(ParticleIdx);
			}
		}
	);
}

void ARopeSimulatorCPU::SolveVelocity(float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount)
{
	// 確定したPositionsから一旦Velocitiesを決定する
	float InvSubStepDeltaSeconds = 1.0f / SubStepDeltaSeconds;
	float InvSquareSubStepDeltaSeconds = InvSubStepDeltaSeconds * InvSubStepDeltaSeconds;
	for (int32 ParticleIdx = 0; ParticleIdx < NumRopes; ++ParticleIdx)
	{
		// 位置コンストレイントで受ける力の総和を計算。コリジョンの押し返される反作用力と形状維持コンストレイントの実効力。
		// Accelerations変数を外力の加速度として使うだけでなく、位置コンストレイントで受ける力の総和として再利用する。
		Accelerations[ParticleIdx] = (Positions[ParticleIdx] - PrevConstraintSolvePositions[ParticleIdx]) * InvSquareSubStepDeltaSeconds;

		PrevConstraintSolveVelocities[ParticleIdx] = Velocities[ParticleIdx]; // 位置コンストレイントと静止摩擦で位置が修正される前の速度。反発の計算に使う
		Velocities[ParticleIdx] = (Positions[ParticleIdx] - PrevPositions[ParticleIdx]) * InvSubStepDeltaSeconds;
		PrevSolveVelocities[ParticleIdx] = Velocities[ParticleIdx]; // 位置コンストレイントと静止摩擦を反映した速度
	}

	if (CollisionRestitution > KINDA_SMALL_NUMBER || CollisionDynamicFriction > KINDA_SMALL_NUMBER || WallRestitution > KINDA_SMALL_NUMBER || WallDynamicFriction > KINDA_SMALL_NUMBER)
	{
		ParallelFor(NumThreads,
			[DeltaSeconds, SubStepDeltaSeconds, SubStepCount, this](int32 ThreadIndex)
			{
				for (int32 ParticleIdx = NumThreadParticles * ThreadIndex; ParticleIdx < NumThreadParticles * (ThreadIndex + 1) && ParticleIdx < NumRopes; ++ParticleIdx)
				{
					if (CollisionRestitution > KINDA_SMALL_NUMBER || CollisionDynamicFriction > KINDA_SMALL_NUMBER)
					{
						ApplyCollisionActorsVelocityConstraint(ParticleIdx, DeltaSeconds, SubStepDeltaSeconds, SubStepCount);
						ApplyCoinBlockersVelocityConstraint(ParticleIdx, DeltaSeconds, SubStepDeltaSeconds, SubStepCount);
					}

					if (WallRestitution > KINDA_SMALL_NUMBER || WallDynamicFriction > KINDA_SMALL_NUMBER)
					{
						ApplyWallVelocityConstraint(ParticleIdx, SubStepDeltaSeconds);
					}
				}
			}
		);
	}
}

namespace
{
	bool CalculateRopeToPlaneCollision(const FPlane& RopePlane, const FVector& RopeCenter, float RopeRadius, const FPlane& Plane, FVector& OutDeepestPenetratePoint, float& OutDeepestPenetrateDepth)
	{
		FVector PlaneIntersectionLineBasePoint;
		FVector PlaneIntersectionLineDir;
		bool bIntersecting = FMath::IntersectPlanes2(PlaneIntersectionLineBasePoint, PlaneIntersectionLineDir, RopePlane, Plane);
		if (bIntersecting)
		{
			FVector ClosestPoint;
			// 戻り値は使わない
			// float RopeCenterToIntersectionLineLength =
			FMath::PointDistToLine(RopeCenter, PlaneIntersectionLineDir, PlaneIntersectionLineBasePoint, ClosestPoint);

			if (Plane.PlaneDot(RopeCenter) < 0.0f)
			{
				OutDeepestPenetratePoint = RopeCenter + (RopeCenter - ClosestPoint).GetSafeNormal() * RopeRadius;
			}
			else
			{
				OutDeepestPenetratePoint = RopeCenter + (ClosestPoint - RopeCenter).GetSafeNormal() * RopeRadius;
			}

			OutDeepestPenetrateDepth = -Plane.PlaneDot(OutDeepestPenetratePoint);
		}
		else
		{
			OutDeepestPenetratePoint = RopeCenter;
			OutDeepestPenetrateDepth = -Plane.PlaneDot(RopeCenter);
		}

		return (OutDeepestPenetrateDepth > 0.0f);
	}
}

void ARopeSimulatorCPU::ApplyCollisionActorsConstraint(int32 ParticleIdx, int32 InFrameExeCount)
{
	float FrameExeAlpha = (InFrameExeCount + 1) / (NumSubStep * NumIteration);

	const FVector& RopeCenter = Positions[ParticleIdx];

#if 0 // TODO:DiskからSphere形状への変化は後で修正
	// TODO:コリジョンについても、NeighborGridに入ってるものだけを処理するようにすればコストは減らせる。まあ数がパーティクルほど多くないし複数のセルにまたがるものが多いだろうからやるのも微妙だが
	// そもそも物理アセット、あるいはプレイヤーのカプセルがこのアクタと交差してなければコリジョン計算の必要すらない。
	// TODO:現フレームの現イテレーションでのコリジョンの位置と向きを前フレームのポーズでのコリジョン位置と現フレームのポーズでのコリジョン位置から
	// 補間で求めているが、それをパーティクルごとにやるのはもったいない。キャッシュしたいね。キャッシュもイテレーションごとにやる必要出るが。
	for (int32 i = 0; i < ActorsAggGeom.Num(); ++i)
	{
		const FKAggregateGeom& AggGeom = ActorsAggGeom[i];
		const FKAggregateGeom& PrevAggGeom = PrevActorsAggGeom[i];

		for (int32 j = 0; j < AggGeom.SphereElems.Num(); ++j)
		{
			// FKSphereElem::GetClosestPointAndNormalを参考にしている
			const FKSphereElem& SphereElem = AggGeom.SphereElems[j];
			const FKSphereElem& PrevSphereElem = PrevAggGeom.SphereElems[j];

			// イテレーションごとの位置補間は線形補間で行う
			const FVector& CollisionCenter = FMath::Lerp(PrevSphereElem.Center, SphereElem.Center, FrameExeAlpha);

			const FVector& CollisionCenterProjectedPoint = FVector::PointPlaneProject(CollisionCenter, RopePlane);

			// これは厳密には球への最近接点でない。球がRopeの面の中央部分に接した場合などはRopeの円周部分は
			// 最近接点にならない。球がRopeより十分大きいという前提での近似になっている。
			const FVector& ClosestRopePoint = RopeCenter + (CollisionCenterProjectedPoint - RopeCenter).GetSafeNormal() * RopeRadius;

			//TODO: RopeCenterと球のコリジョンをとらなくていい？

			if ((ClosestRopePoint - CollisionCenter).SizeSquared() < SphereElem.Radius * SphereElem.Radius)
			{
				FVector DeltaPos = (ClosestRopePoint - CollisionCenter).GetSafeNormal() * (SphereElem.Radius - (ClosestRopePoint - CollisionCenter).Size()) * CollisionProjectionAlpha;
				if (((ClosestRopePoint - CollisionCenter) | (RopeCenter - CollisionCenter)) > 0.0f)
				{
					DeltaPos = (ClosestRopePoint - CollisionCenter).GetSafeNormal() * (SphereElem.Radius - (ClosestRopePoint - CollisionCenter).Size()) * CollisionProjectionAlpha;
				}
				else
				{
					// もし大きくめりこんで、RopeCenterに対してDeepestPenetratePointがカプセルの軸の反対側になったら
					// 反発方向はRopeCenter側で反発距離にSphereRadiusを加算する
					DeltaPos = (CollisionCenter - ClosestRopePoint).GetSafeNormal() * (SphereElem.Radius + (CollisionCenter - ClosestRopePoint).Size()) * CollisionProjectionAlpha;
				}
				Positions[ParticleIdx] += DeltaPos;
			}
		}

		for (int32 j = 0; j < AggGeom.BoxElems.Num(); ++j)
		{
			const FKBoxElem& BoxElem = AggGeom.BoxElems[j];

			FTransform BoxTM = PrevAggGeom.BoxElems[j].GetTransform();
			// イテレーションごとの位置補間は線形補間で行う
			BoxTM.BlendWith(BoxElem.GetTransform(), FrameExeAlpha);

			FVector HalfExtent(0.5f * BoxElem.X, 0.5f * BoxElem.Y, 0.5f * BoxElem.Z);
			const FVector& BoxMin = BoxTM.TransformPositionNoScale(-HalfExtent);
			const FVector& BoxMax = BoxTM.TransformPositionNoScale(HalfExtent);

			const FVector& BoxZAxis = BoxTM.GetUnitAxis(EAxis::Type::Z);
			const FVector& BoxXAxis = BoxTM.GetUnitAxis(EAxis::Type::X);
			const FVector& BoxYAxis = BoxTM.GetUnitAxis(EAxis::Type::Y);

			// CalculateRopeToPlaneCollisionの戻り値はみない。DeepestPenetrateDepth > 0.0fならコリジョンしているので
			FVector DeepestPenetratePointMinZ;
			float DeepestPenetrateDepthMinZ = 0.0f;
			bool bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMin, -BoxZAxis), DeepestPenetratePointMinZ, DeepestPenetrateDepthMinZ);
			if (!bCollisioned)
			{
				continue;
			}

			FVector DeepestPenetratePointMaxZ;
			float DeepestPenetrateDepthMaxZ = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMax, BoxZAxis), DeepestPenetratePointMaxZ, DeepestPenetrateDepthMaxZ);
			if (!bCollisioned)
			{
				continue;
			}

			FVector DeepestPenetratePointMinX;
			float DeepestPenetrateDepthMinX = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMin, -BoxXAxis), DeepestPenetratePointMinX, DeepestPenetrateDepthMinX);
			if (!bCollisioned)
			{
				continue;
			}


			FVector DeepestPenetratePointMaxX;
			float DeepestPenetrateDepthMaxX = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMax, BoxXAxis), DeepestPenetratePointMaxX, DeepestPenetrateDepthMaxX);
			if (!bCollisioned)
			{
				continue;
			}


			FVector DeepestPenetratePointMinY;
			float DeepestPenetrateDepthMinY = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMin, -BoxYAxis), DeepestPenetratePointMinY, DeepestPenetrateDepthMinY);
			if (!bCollisioned)
			{
				continue;
			}

			FVector DeepestPenetratePointMaxY;
			float DeepestPenetrateDepthMaxY = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMax, BoxYAxis), DeepestPenetratePointMaxY, DeepestPenetrateDepthMaxY);
			if (!bCollisioned)
			{
				continue;
			}

			check(DeepestPenetrateDepthMinZ > 0.0f);
			check(DeepestPenetrateDepthMaxZ > 0.0f);
			check(DeepestPenetrateDepthMinX > 0.0f);
			check(DeepestPenetrateDepthMaxX > 0.0f);
			check(DeepestPenetrateDepthMinY > 0.0f);
			check(DeepestPenetrateDepthMaxY > 0.0f);

			// 6個の変数をifで大小比較するのが大変なので3個にしぼる
			// 配列にしてソートしてもいいが、compute shaderで扱うのが面倒なので
			float SmallerPenetrateDepthZ = DeepestPenetrateDepthMinZ;
			bool bSmallerIsMinZ = true;
			if (DeepestPenetrateDepthMaxZ < DeepestPenetrateDepthMinZ)
			{
				SmallerPenetrateDepthZ = DeepestPenetrateDepthMaxZ;
				bSmallerIsMinZ = false;
			}

			float SmallerPenetrateDepthX = DeepestPenetrateDepthMinX;
			bool bSmallerIsMinX = true;
			if (DeepestPenetrateDepthMaxX < DeepestPenetrateDepthMinX)
			{
				SmallerPenetrateDepthX = DeepestPenetrateDepthMaxX;
				bSmallerIsMinX = false;
			}

			float SmallerPenetrateDepthY = DeepestPenetrateDepthMinY;
			bool bSmallerIsMinY = true;
			if (DeepestPenetrateDepthMaxY < DeepestPenetrateDepthMinY)
			{
				SmallerPenetrateDepthY = DeepestPenetrateDepthMaxY;
				bSmallerIsMinY = false;
			}

			float SmallestPenetrateDepth;
			FVector SmallestPenetratePoint;
			FPlane SmallestBoxPlane;

			if (SmallerPenetrateDepthX < SmallerPenetrateDepthY)
			{
				if (SmallerPenetrateDepthX < SmallerPenetrateDepthZ)
				{
					SmallestPenetrateDepth = SmallerPenetrateDepthX;
					SmallestPenetratePoint = bSmallerIsMinX ? DeepestPenetratePointMinX : DeepestPenetratePointMaxX;
					SmallestBoxPlane = bSmallerIsMinX ? FPlane(BoxMin, -BoxXAxis) : FPlane(BoxMax, BoxXAxis);
				}
				else
				{
					SmallestPenetrateDepth = SmallerPenetrateDepthZ;
					SmallestPenetratePoint = bSmallerIsMinZ ? DeepestPenetratePointMinZ : DeepestPenetratePointMaxZ;
					SmallestBoxPlane = bSmallerIsMinZ ? FPlane(BoxMin, -BoxZAxis) : FPlane(BoxMax, BoxZAxis);
				}
			}
			else // SmallerPenetrateDepthY <= SmallerPenetrateDepthX
			{
				if (SmallerPenetrateDepthY < SmallerPenetrateDepthZ)
				{
					SmallestPenetrateDepth = SmallerPenetrateDepthY;
					SmallestPenetratePoint = bSmallerIsMinY ? DeepestPenetratePointMinY : DeepestPenetratePointMaxY;
					SmallestBoxPlane = bSmallerIsMinY ? FPlane(BoxMin, -BoxYAxis) : FPlane(BoxMax, BoxYAxis);
				}
				else
				{
					SmallestPenetrateDepth = SmallerPenetrateDepthZ;
					SmallestPenetratePoint = bSmallerIsMinZ ? DeepestPenetratePointMinZ : DeepestPenetratePointMaxZ;
					SmallestBoxPlane = bSmallerIsMinZ ? FPlane(BoxMin, -BoxZAxis) : FPlane(BoxMax, BoxZAxis);
				}
			}

			SmallestPenetrateDepth *= CollisionProjectionAlpha;

			// RopeCenterがBoxの内側にあるかどうかは考慮しなくてもさほど影響は出ないだろうという妥協
			float CenterProjection = FMath::Max(-SmallestBoxPlane.PlaneDot(RopeCenter) * CollisionProjectionAlpha, 0.0f);
			const FVector& DeltaPos = CenterProjection * SmallestBoxPlane.GetNormal();
			Positions[ParticleIdx] += DeltaPos;
		}

		for (int32 j = 0; j < AggGeom.SphylElems.Num(); ++j)
		{
			// FKSphylElem::GetClosestPointAndNormalを参考にしている
			const FKSphylElem& SphylElem = AggGeom.SphylElems[j];
			const FKSphylElem& PrevSphylElem = PrevAggGeom.SphylElems[j];

			// イテレーションごとの位置補間は線形補間で行う
			const FVector& CollisionCenter = FMath::Lerp(PrevSphylElem.Center, SphylElem.Center, FrameExeAlpha);

			FTransform CapsuleTM = PrevSphylElem.GetTransform();
			// イテレーションごとの位置補間は線形補間で行う
			CapsuleTM.BlendWith(SphylElem.GetTransform(), FrameExeAlpha);

			const FVector& StartPoint = CollisionCenter + CapsuleTM.GetUnitAxis(EAxis::Type::Z) * SphylElem.Length * 0.5f;
			const FVector& EndPoint = CollisionCenter + CapsuleTM.GetUnitAxis(EAxis::Type::Z) * SphylElem.Length * -0.5f;
			// FMath::PointDistToSegmentSquared()は中でFMath::ClosestPointOnSegment() を呼び出しているので下で2回呼び出すのは無駄なので
			// FMath::ClosestPointOnSegment()を使う
			//float DistSquared = FMath::PointDistToSegmentSquared(Positions[ParticleIdx], StartPoint, EndPoint);
			FVector ClosestPointOnSegment = FMath::ClosestPointOnSegment(RopeCenter, StartPoint, EndPoint);
			const FVector& ClosestPointOnSegmentProjected = FVector::PointPlaneProject(ClosestPointOnSegment, RopePlane);
			const FVector& DeepestPenetratePoint = RopeCenter + (ClosestPointOnSegmentProjected - RopeCenter).GetSafeNormal() * RopeRadius;

			// 厳密にはこのClosestPointOnSegmentとRopeCenterから計算した上のClosestPointOnSegmentは違うが、
			// カプセルがRopeと比較してサイズが大きいものとして近似する。
			ClosestPointOnSegment = FMath::ClosestPointOnSegment(DeepestPenetratePoint, StartPoint, EndPoint);
			if ((ClosestPointOnSegment - DeepestPenetratePoint).SizeSquared() < SphylElem.Radius * SphylElem.Radius)
			{
				FVector DeltaPos = (DeepestPenetratePoint - ClosestPointOnSegment).GetSafeNormal() * (SphylElem.Radius - (ClosestPointOnSegment - DeepestPenetratePoint).Size()) * CollisionProjectionAlpha;
				if (((ClosestPointOnSegment - DeepestPenetratePoint) | (ClosestPointOnSegment - RopeCenter)) > 0.0f)
				{
					DeltaPos = (DeepestPenetratePoint - ClosestPointOnSegment).GetSafeNormal() * (SphylElem.Radius - (ClosestPointOnSegment - DeepestPenetratePoint).Size()) * CollisionProjectionAlpha;
				}
				else
				{
					// もし大きくめりこんで、RopeCenterに対してDeepestPenetratePointがカプセルの軸の反対側になったら
					// 反発方向はRopeCenter側で反発距離にSphereRadiusを加算する
					DeltaPos = (ClosestPointOnSegment - DeepestPenetratePoint).GetSafeNormal() * (SphylElem.Radius + (ClosestPointOnSegment - DeepestPenetratePoint).Size()) * CollisionProjectionAlpha;
				}
				Positions[ParticleIdx] += DeltaPos;
			}
		}
	}
#endif
}

void ARopeSimulatorCPU::ApplyCoinBlockersCollisionConstraint(int32 ParticleIdx, int32 InFrameExeCount)
{
	float FrameExeAlpha = (InFrameExeCount + 1) / (NumSubStep * NumIteration);

	const FVector& RopeCenter = Positions[ParticleIdx];

#if 0 // TODO:DiskからSphere形状への変化は後で修正
	for (TPair<TWeakObjectPtr<class UPrimitiveComponent>, FTransform> CollisionPose : CoinBlockerCollisionsPoseMap)
	{
		if (CollisionPose.Key == nullptr)
		{
			continue;
		}

		const FTransform& Pose = CollisionPose.Value;
		// 前フレームで同じコンポーネントがなければ、前フレームのポーズは現フレームのポーズとしておく
		const FTransform* PrevPosePtr = PrevCoinBlockerCollisionsPoseMap.Find(CollisionPose.Key);
		const FTransform& PrevPose = PrevPosePtr == nullptr ? Pose : *PrevPosePtr;
		// イテレーションごとの位置補間は線形補間で行う
		const FVector& CollisionCenter = FMath::Lerp(PrevPose.GetLocation(), Pose.GetLocation(), FrameExeAlpha);

		// USphereComponentとUCapsuleComponent以外は今のところ非対応
		const USphereComponent* SphereComp = Cast<const USphereComponent>(CollisionPose.Key.Get());
		if (SphereComp != nullptr)
		{
			const FVector& CollisionCenterProjectedPoint = FVector::PointPlaneProject(CollisionCenter, RopePlane);

			// これは厳密には球への最近接点でない。球がRopeの面の中央部分に接した場合などはRopeの円周部分は
			// 最近接点にならない。球がRopeより十分大きいという前提での近似になっている。
			const FVector& ClosestRopePoint = RopeCenter + (CollisionCenterProjectedPoint - RopeCenter).GetSafeNormal() * RopeRadius;

			//TODO: RopeCenterと球のコリジョンをとらなくていい？

			float SphereRadius = SphereComp->GetScaledSphereRadius();
			if ((ClosestRopePoint - CollisionCenter).SizeSquared() < SphereRadius * SphereRadius)
			{
				FVector DeltaPos = (ClosestRopePoint - CollisionCenter).GetSafeNormal() * (SphereRadius - (ClosestRopePoint - CollisionCenter).Size()) * CollisionProjectionAlpha;
				if (((ClosestRopePoint - CollisionCenter) | (RopeCenter - CollisionCenter)) > 0.0f)
				{
					DeltaPos = (ClosestRopePoint - CollisionCenter).GetSafeNormal() * (SphereRadius - (ClosestRopePoint - CollisionCenter).Size()) * CollisionProjectionAlpha;
				}
				else
				{
					// もし大きくめりこんで、RopeCenterに対してDeepestPenetratePointがカプセルの軸の反対側になったら
					// 反発方向はRopeCenter側で反発距離にSphereRadiusを加算する
					DeltaPos = (CollisionCenter - ClosestRopePoint).GetSafeNormal() * (SphereRadius + (CollisionCenter - ClosestRopePoint).Size()) * CollisionProjectionAlpha;
				}
				Positions[ParticleIdx] += DeltaPos;
			}
			continue;
		}

		UCapsuleComponent* CapsuleComp = Cast<UCapsuleComponent>(CollisionPose.Key.Get());
		if (CapsuleComp != nullptr)
		{
			FTransform CapsuleTM = Pose;
			// イテレーションごとの位置補間は線形補間で行う
			CapsuleTM.BlendWith(PrevPose, FrameExeAlpha);


			float HalfHeight = CapsuleComp->GetScaledCapsuleHalfHeight_WithoutHemisphere();
			const FVector& ZAxis = CapsuleTM.GetUnitAxis(EAxis::Type::Z);
			const FVector& StartPoint = CollisionCenter + ZAxis * HalfHeight;
			const FVector& EndPoint = CollisionCenter - ZAxis * HalfHeight;

			// FMath::PointDistToSegmentSquared()は中でFMath::ClosestPointOnSegment() を呼び出しているので下で2回呼び出すのは無駄なので
			// FMath::ClosestPointOnSegment()を使う
			//float DistSquared = FMath::PointDistToSegmentSquared(Positions[ParticleIdx], StartPoint, EndPoint);
			FVector ClosestPointOnSegment = FMath::ClosestPointOnSegment(RopeCenter, StartPoint, EndPoint);
			const FVector& ClosestPointOnSegmentProjected = FVector::PointPlaneProject(ClosestPointOnSegment, RopePlane);
			const FVector& DeepestPenetratePoint = RopeCenter + (ClosestPointOnSegmentProjected - RopeCenter).GetSafeNormal() * RopeRadius;

			float CapsuleRadius = CapsuleComp->GetScaledCapsuleRadius();
			// 厳密にはこのClosestPointOnSegmentとRopeCenterから計算した上のClosestPointOnSegmentは違うが、
			// カプセルがRopeと比較してサイズが大きいものとして近似する。
			ClosestPointOnSegment = FMath::ClosestPointOnSegment(DeepestPenetratePoint, StartPoint, EndPoint);
			if ((ClosestPointOnSegment - DeepestPenetratePoint).SizeSquared() < CapsuleRadius * CapsuleRadius)
			{
				FVector DeltaPos = (DeepestPenetratePoint - ClosestPointOnSegment).GetSafeNormal() * (CapsuleRadius - (ClosestPointOnSegment - DeepestPenetratePoint).Size()) * CollisionProjectionAlpha;
				if (((ClosestPointOnSegment - DeepestPenetratePoint) | (ClosestPointOnSegment - RopeCenter)) > 0.0f)
				{
					DeltaPos = (DeepestPenetratePoint - ClosestPointOnSegment).GetSafeNormal() * (CapsuleRadius - (ClosestPointOnSegment - DeepestPenetratePoint).Size()) * CollisionProjectionAlpha;
				}
				else
				{
					// もし大きくめりこんで、RopeCenterに対してDeepestPenetratePointがカプセルの軸の反対側になったら
					// 反発方向はRopeCenter側で反発距離にSphereRadiusを加算する
					DeltaPos = (ClosestPointOnSegment - DeepestPenetratePoint).GetSafeNormal() * (CapsuleRadius + (ClosestPointOnSegment - DeepestPenetratePoint).Size()) * CollisionProjectionAlpha;
				}
				Positions[ParticleIdx] += DeltaPos;
			}
			continue;
		}
	}
#endif
}

void ARopeSimulatorCPU::CalculateOneWallCollisionProjection(int32 ParticleIdx, const FPlane& WallPlane, FVector& InOutDeltaPos, FQuat& InOutDeltaRot)
{
	const FVector& RopeCenter = Positions[ParticleIdx];

	float CenterProjection = -WallPlane.PlaneDot(RopeCenter) * WallProjectionAlpha;

	// ブレークポイントをはれるようにFMath::Max()でなくifを使っている
	if (CenterProjection > 0.0f)
	{
		InOutDeltaPos += CenterProjection * WallPlane.GetNormal();
	}
	else
	{
		// 下の計算で使うのでめりこんでないときは0に
		CenterProjection = 0.0f;
	}

#if 0 // TODO:DiskからSphere形状への変化は後で修正
	FVector DeepestPenetratePoint;
	float DeepestPenetrateDepth;
	bool bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, WallPlane, DeepestPenetratePoint, DeepestPenetrateDepth);
	DeepestPenetrateDepth *= WallProjectionAlpha;
#endif
}

void ARopeSimulatorCPU::ApplyWallCollisionConstraint(int32 ParticleIdx)
{
	FVector DeltaPos = FVector::ZeroVector;
	FQuat DeltaRot = FQuat(0.0f, 0.0f, 0.0f, 0.0f);

	// MinZ
	CalculateOneWallCollisionProjection(ParticleIdx, FPlane(WallBox.Min, FVector::ZAxisVector), DeltaPos, DeltaRot);
	// MaxZ
	CalculateOneWallCollisionProjection(ParticleIdx, FPlane(WallBox.Max, -FVector::ZAxisVector), DeltaPos, DeltaRot);
	// MinX
	CalculateOneWallCollisionProjection(ParticleIdx, FPlane(WallBox.Min, FVector::XAxisVector), DeltaPos, DeltaRot);
	// MaxX
	CalculateOneWallCollisionProjection(ParticleIdx, FPlane(WallBox.Max, -FVector::XAxisVector), DeltaPos, DeltaRot);
	// MinY
	CalculateOneWallCollisionProjection(ParticleIdx, FPlane(WallBox.Min, FVector::YAxisVector), DeltaPos, DeltaRot);
	// MaxY
	CalculateOneWallCollisionProjection(ParticleIdx, FPlane(WallBox.Max, -FVector::YAxisVector), DeltaPos, DeltaRot);

	Positions[ParticleIdx] += DeltaPos;
}

namespace
{
	void SolveRestituionDeltaVelocity(const FVector& RopeCenter, const FVector& DeepestPenetratePoint, const FVector& ImpactNormal, float Restitution, float RestitutionSleepVelocity, const FVector& Velocity, const FVector& ImpactPointVelocity, const FVector& PrevConstraintSolveVelocity, FVector& InOutDeltaVelocity)
	{
		const FVector& RopeCenterToDeepestPenetratePoint = DeepestPenetratePoint - RopeCenter;

		// 外積オペレータ^は演算順序が+より優先ではないのに注意
		const FVector& DeepestPenetratePointRelativeVelocity = Velocity - ImpactPointVelocity;
		float RelativeVelocityNormal = DeepestPenetratePointRelativeVelocity | ImpactNormal;
		if (FMath::Abs(RelativeVelocityNormal) > RestitutionSleepVelocity)
		{
			const FVector& DeepestPenetratePointPrevConstraintSolveRelativeVelocity = PrevConstraintSolveVelocity - ImpactPointVelocity;
			float PrevConstraintSolveRelativeVelocityNormal = DeepestPenetratePointPrevConstraintSolveRelativeVelocity | ImpactNormal;
			const FVector& DeltaVelocity = ImpactNormal * (-RelativeVelocityNormal + FMath::Max(-Restitution * PrevConstraintSolveRelativeVelocityNormal, 0.0f));

			InOutDeltaVelocity += DeltaVelocity;
		}
	}

	void SolveDynamicFrictionDeltaVelocity(const FVector& RopeCenter, const FVector& DeepestPenetratePoint, const FVector& ImpactNormal, float WallDynamicFriction, const FVector& Velocity, const FVector& ImpactPointVelocity, const FVector& Acceleration, float SubStepDeltaSeconds, FVector& InOutDeltaVelocity)
	{
		const FVector& RopeCenterToDeepestPenetratePoint = DeepestPenetratePoint - RopeCenter;

		// 外積オペレータ^は演算順序が+より優先ではないのに注意
		const FVector& DeepestPenetratePointRelativeVelocity = Velocity - ImpactPointVelocity;
		float RelativeVelocityNormal = DeepestPenetratePointRelativeVelocity | ImpactNormal;

		const FVector& RelativeVelocityTangent = Velocity - RelativeVelocityNormal * ImpactNormal;
		float ForceNormalLen = FMath::Abs(Acceleration | ImpactNormal);
		float RelativeVelocityTangentLen = RelativeVelocityTangent.Size();
		const FVector& DeltaVelocity = -RelativeVelocityTangent / FMath::Max(RelativeVelocityTangentLen, KINDA_SMALL_NUMBER) * FMath::Min(SubStepDeltaSeconds * WallDynamicFriction * ForceNormalLen, RelativeVelocityTangentLen);

		InOutDeltaVelocity += DeltaVelocity;
	}
}

void ARopeSimulatorCPU::ApplyCollisionActorsVelocityConstraint(int32 ParticleIdx, float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount)
{
	float RestitutionSleepVelocity = 2.0f * FMath::Abs(Gravity) * SubStepDeltaSeconds;
	float SubStepAlpha = (SubStepCount + 1) / NumSubStep;

	const FVector& RopeCenter = Positions[ParticleIdx];

	const FVector& PrevSolveVelocity = PrevSolveVelocities[ParticleIdx];
	const FVector& PrevConstraintSolveVelocity = PrevConstraintSolveVelocities[ParticleIdx];
	const FVector& Velocity = Velocities[ParticleIdx];
	const FVector& Acceleration = Accelerations[ParticleIdx];

	FVector DeltaVelocity = FVector::ZeroVector;

#if 0 // TODO:DiskからSphere形状への変化は後で修正
	// TODO:コリジョンについても、NeighborGridに入ってるものだけを処理するようにすればコストは減らせる。まあ数がパーティクルほど多くないし複数のセルにまたがるものが多いだろうからやるのも微妙だが
	// そもそも物理アセット、あるいはプレイヤーのカプセルがこのアクタと交差してなければコリジョン計算の必要すらない。
	// TODO:現フレームの現イテレーションでのコリジョンの位置と向きを前フレームのポーズでのコリジョン位置と現フレームのポーズでのコリジョン位置から
	// 補間で求めているが、それをパーティクルごとにやるのはもったいない。キャッシュしたいね。キャッシュもイテレーションごとにやる必要出るが。
	for (int32 i = 0; i < ActorsAggGeom.Num(); ++i)
	{
		const FKAggregateGeom& AggGeom = ActorsAggGeom[i];
		const FKAggregateGeom& PrevAggGeom = PrevActorsAggGeom[i];

		for (int32 j = 0; j <AggGeom.SphereElems.Num(); ++j)
		{
			// FKSphereElem::GetClosestPointAndNormalを参考にしている
			const FKSphereElem& SphereElem = AggGeom.SphereElems[j];
			const FKSphereElem& PrevSphereElem = PrevAggGeom.SphereElems[j];

			// イテレーションごとの位置補間は線形補間で行う
			const FVector& CollisionCenter = FMath::Lerp(PrevSphereElem.Center, SphereElem.Center, SubStepAlpha);

			const FVector& CollisionCenterProjectedPoint = FVector::PointPlaneProject(CollisionCenter, RopePlane);

			// これは厳密には球への最近接点でない。球がRopeの面の中央部分に接した場合などはRopeの円周部分は
			// 最近接点にならない。球がRopeより十分大きいという前提での近似になっている。
			const FVector& ClosestRopePoint = RopeCenter + (CollisionCenterProjectedPoint - RopeCenter).GetSafeNormal() * RopeRadius;

			//TODO: RopeCenterと球のコリジョンをとらなくていい？

			if ((ClosestRopePoint - CollisionCenter).SizeSquared() < SphereElem.Radius * SphereElem.Radius)
			{
				const FVector& CollisionVelocity = (SphereElem.Center - PrevSphereElem.Center) / DeltaSeconds;

				FVector ImpactNormal = (RopeCenter - CollisionCenter).GetSafeNormal();
				if (((ClosestRopePoint - CollisionCenter) | (RopeCenter - CollisionCenter)) < 0.0f)
				{
					ImpactNormal *= -1;
				}

				if (CollisionRestitution > KINDA_SMALL_NUMBER)
				{
					SolveRestituionDeltaVelocity(RopeCenter, ClosestRopePoint, ImpactNormal, CollisionRestitution, RestitutionSleepVelocity, PrevSolveVelocity, CollisionVelocity, PrevConstraintSolveVelocity, DeltaVelocity);
				}

				if (CollisionDynamicFriction > KINDA_SMALL_NUMBER)
				{
					SolveDynamicFrictionDeltaVelocity(RopeCenter, ClosestRopePoint, ImpactNormal, CollisionDynamicFriction, PrevSolveVelocity, CollisionVelocity, Acceleration, SubStepDeltaSeconds, DeltaVelocity);
				}
			}
		}

		for (int32 j = 0; j <AggGeom.BoxElems.Num(); ++j)
		{
			const FKBoxElem& BoxElem = AggGeom.BoxElems[j];

			FTransform BoxTM = PrevAggGeom.BoxElems[j].GetTransform();
			// イテレーションごとの位置補間は線形補間で行う
			BoxTM.BlendWith(BoxElem.GetTransform(), SubStepAlpha);

			FVector HalfExtent(0.5f * BoxElem.X, 0.5f * BoxElem.Y, 0.5f * BoxElem.Z);
			const FVector& BoxMin = BoxTM.TransformPositionNoScale(-HalfExtent);
			const FVector& BoxMax = BoxTM.TransformPositionNoScale(HalfExtent);

			const FVector& BoxZAxis = BoxTM.GetUnitAxis(EAxis::Type::Z);
			const FVector& BoxXAxis = BoxTM.GetUnitAxis(EAxis::Type::X);
			const FVector& BoxYAxis = BoxTM.GetUnitAxis(EAxis::Type::Y);

			// CalculateRopeToPlaneCollisionの戻り値はみない。DeepestPenetrateDepth > 0.0fならコリジョンしているので
			FVector DeepestPenetratePointMinZ;
			float DeepestPenetrateDepthMinZ = 0.0f;
			bool bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMin, -BoxZAxis), DeepestPenetratePointMinZ, DeepestPenetrateDepthMinZ);
			if (!bCollisioned)
			{
				continue;
			}

			FVector DeepestPenetratePointMaxZ;
			float DeepestPenetrateDepthMaxZ = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMax, BoxZAxis), DeepestPenetratePointMaxZ, DeepestPenetrateDepthMaxZ);
			if (!bCollisioned)
			{
				continue;
			}

			FVector DeepestPenetratePointMinX;
			float DeepestPenetrateDepthMinX = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMin, -BoxXAxis), DeepestPenetratePointMinX, DeepestPenetrateDepthMinX);
			if (!bCollisioned)
			{
				continue;
			}


			FVector DeepestPenetratePointMaxX;
			float DeepestPenetrateDepthMaxX = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMax, BoxXAxis), DeepestPenetratePointMaxX, DeepestPenetrateDepthMaxX);
			if (!bCollisioned)
			{
				continue;
			}


			FVector DeepestPenetratePointMinY;
			float DeepestPenetrateDepthMinY = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMin, -BoxYAxis), DeepestPenetratePointMinY, DeepestPenetrateDepthMinY);
			if (!bCollisioned)
			{
				continue;
			}

			FVector DeepestPenetratePointMaxY;
			float DeepestPenetrateDepthMaxY = 0.0f;
			bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, FPlane(BoxMax, BoxYAxis), DeepestPenetratePointMaxY, DeepestPenetrateDepthMaxY);
			if (!bCollisioned)
			{
				continue;
			}

			check(DeepestPenetrateDepthMinZ > 0.0f);
			check(DeepestPenetrateDepthMaxZ > 0.0f);
			check(DeepestPenetrateDepthMinX > 0.0f);
			check(DeepestPenetrateDepthMaxX > 0.0f);
			check(DeepestPenetrateDepthMinY > 0.0f);
			check(DeepestPenetrateDepthMaxY > 0.0f);

			// 6個の変数をifで大小比較するのが大変なので3個にしぼる
			// 配列にしてソートしてもいいが、compute shaderで扱うのが面倒なので
			float SmallerPenetrateDepthZ = DeepestPenetrateDepthMinZ;
			bool bSmallerIsMinZ = true;
			if (DeepestPenetrateDepthMaxZ < DeepestPenetrateDepthMinZ)
			{
				SmallerPenetrateDepthZ = DeepestPenetrateDepthMaxZ;
				bSmallerIsMinZ = false;
			}

			float SmallerPenetrateDepthX = DeepestPenetrateDepthMinX;
			bool bSmallerIsMinX = true;
			if (DeepestPenetrateDepthMaxX < DeepestPenetrateDepthMinX)
			{
				SmallerPenetrateDepthX = DeepestPenetrateDepthMaxX;
				bSmallerIsMinX = false;
			}

			float SmallerPenetrateDepthY = DeepestPenetrateDepthMinY;
			bool bSmallerIsMinY = true;
			if (DeepestPenetrateDepthMaxY < DeepestPenetrateDepthMinY)
			{
				SmallerPenetrateDepthY = DeepestPenetrateDepthMaxY;
				bSmallerIsMinY = false;
			}

			FVector SmallestPenetratePoint;
			FPlane SmallestBoxPlane;
			if (SmallerPenetrateDepthX < SmallerPenetrateDepthY)
			{
				if (SmallerPenetrateDepthX < SmallerPenetrateDepthZ)
				{
					SmallestPenetratePoint = bSmallerIsMinX ? DeepestPenetratePointMinX : DeepestPenetratePointMaxX;
					SmallestBoxPlane = bSmallerIsMinX ? FPlane(BoxMin, -BoxXAxis) : FPlane(BoxMax, BoxXAxis);
				}
				else
				{
					SmallestPenetratePoint = bSmallerIsMinZ ? DeepestPenetratePointMinZ : DeepestPenetratePointMaxZ;
					SmallestBoxPlane = bSmallerIsMinZ ? FPlane(BoxMin, -BoxZAxis) : FPlane(BoxMax, BoxZAxis);
				}
			}
			else // SmallerPenetrateDepthY <= SmallerPenetrateDepthX
			{
				if (SmallerPenetrateDepthY < SmallerPenetrateDepthZ)
				{
					SmallestPenetratePoint = bSmallerIsMinY ? DeepestPenetratePointMinY : DeepestPenetratePointMaxY;
					SmallestBoxPlane = bSmallerIsMinY ? FPlane(BoxMin, -BoxYAxis) : FPlane(BoxMax, BoxYAxis);
				}
				else
				{
					SmallestPenetratePoint = bSmallerIsMinZ ? DeepestPenetratePointMinZ : DeepestPenetratePointMaxZ;
					SmallestBoxPlane = bSmallerIsMinZ ? FPlane(BoxMin, -BoxZAxis) : FPlane(BoxMax, BoxZAxis);
				}
			}

			// インパクトポイントの速度を求める
			const FKBoxElem& PrevBoxElem = PrevAggGeom.BoxElems[j];
			const FQuat& RotDiff = BoxElem.GetTransform().GetRotation() * PrevBoxElem.GetTransform().GetRotation().Inverse();
			const FVector& PrevCenterToPenetratePoint = RotDiff.UnrotateVector(SmallestPenetratePoint - BoxElem.Center);
			const FVector& CollisionImpactPointVelocity = (SmallestPenetratePoint - (PrevBoxElem.Center + PrevCenterToPenetratePoint)) / DeltaSeconds;

			if (CollisionRestitution > KINDA_SMALL_NUMBER)
			{
				SolveRestituionDeltaVelocity(RopeCenter, SmallestPenetratePoint, SmallestBoxPlane.GetNormal(), CollisionRestitution, RestitutionSleepVelocity, PrevSolveVelocity, CollisionImpactPointVelocity, PrevConstraintSolveVelocity, DeltaVelocity);
			}

			if (CollisionDynamicFriction > KINDA_SMALL_NUMBER)
			{
				SolveDynamicFrictionDeltaVelocity(RopeCenter, SmallestPenetratePoint, SmallestBoxPlane.GetNormal(), CollisionDynamicFriction, PrevSolveVelocity, CollisionImpactPointVelocity, Acceleration, SubStepDeltaSeconds, DeltaVelocity);
			}
		}

		for (int32 j = 0; j <AggGeom.SphylElems.Num(); ++j)
		{
			// FKSphylElem::GetClosestPointAndNormalを参考にしている
			const FKSphylElem& SphylElem = AggGeom.SphylElems[j];
			const FKSphylElem& PrevSphylElem = PrevAggGeom.SphylElems[j];

			// イテレーションごとの位置補間は線形補間で行う
			const FVector& CollisionCenter = FMath::Lerp(PrevSphylElem.Center, SphylElem.Center, SubStepAlpha);

			FTransform CapsuleTM = PrevSphylElem.GetTransform();
			// イテレーションごとの位置補間は線形補間で行う
			CapsuleTM.BlendWith(SphylElem.GetTransform(), SubStepAlpha);

			const FVector& StartPoint = CollisionCenter + CapsuleTM.GetUnitAxis(EAxis::Type::Z) * SphylElem.Length * 0.5f;
			const FVector& EndPoint = CollisionCenter + CapsuleTM.GetUnitAxis(EAxis::Type::Z) * SphylElem.Length * -0.5f;
			// FMath::PointDistToSegmentSquared()は中でFMath::ClosestPointOnSegment() を呼び出しているので下で2回呼び出すのは無駄なので
			// FMath::ClosestPointOnSegment()を使う
			//float DistSquared = FMath::PointDistToSegmentSquared(Positions[ParticleIdx], StartPoint, EndPoint);
			FVector ClosestPointOnSegment = FMath::ClosestPointOnSegment(RopeCenter, StartPoint, EndPoint);
			const FVector& ClosestPointOnSegmentProjected = FVector::PointPlaneProject(ClosestPointOnSegment, RopePlane);
			const FVector& DeepestPenetratePoint = RopeCenter + (ClosestPointOnSegmentProjected - RopeCenter).GetSafeNormal() * RopeRadius;

			// 厳密にはこのClosestPointOnSegmentとRopeCenterから計算した上のClosestPointOnSegmentは違うが、
			// カプセルがRopeと比較してサイズが大きいものとして近似する。
			ClosestPointOnSegment = FMath::ClosestPointOnSegment(DeepestPenetratePoint, StartPoint, EndPoint);
			if ((ClosestPointOnSegment - DeepestPenetratePoint).SizeSquared() < SphylElem.Radius * SphylElem.Radius)
			{
				// インパクトポイントの速度を求める
				const FQuat& RotDiff = SphylElem.GetTransform().GetRotation() * PrevSphylElem.GetTransform().GetRotation().Inverse();
				const FVector& PrevCenterToDeepestPenetratePoint = RotDiff.UnrotateVector(DeepestPenetratePoint - SphylElem.Center);
				const FVector& CollisionImpactPointVelocity = (DeepestPenetratePoint - (PrevSphylElem.Center + PrevCenterToDeepestPenetratePoint)) / DeltaSeconds;

				FVector ImpactNormal = (DeepestPenetratePoint - ClosestPointOnSegment).GetSafeNormal();
				if (((ClosestPointOnSegment - DeepestPenetratePoint) | (ClosestPointOnSegment - RopeCenter)) < 0.0f)
				{
					ImpactNormal *= -1;
				}

				if (CollisionRestitution > KINDA_SMALL_NUMBER)
				{
					SolveRestituionDeltaVelocity(RopeCenter, DeepestPenetratePoint, ImpactNormal, CollisionRestitution, RestitutionSleepVelocity, PrevSolveVelocity, CollisionImpactPointVelocity, PrevConstraintSolveVelocity, DeltaVelocity);
				}

				if (CollisionDynamicFriction > KINDA_SMALL_NUMBER)
				{
					SolveDynamicFrictionDeltaVelocity(RopeCenter, DeepestPenetratePoint, ImpactNormal, CollisionDynamicFriction, PrevSolveVelocity, CollisionImpactPointVelocity, Acceleration, SubStepDeltaSeconds, DeltaVelocity);
				}
			}
		}
	}

	Velocities[ParticleIdx] += DeltaVelocity;
#endif
}

void ARopeSimulatorCPU::ApplyCoinBlockersVelocityConstraint(int32 ParticleIdx, float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount)
{
	float RestitutionSleepVelocity = 2.0f * FMath::Abs(Gravity) * SubStepDeltaSeconds;
	float SubStepAlpha = (SubStepCount + 1) / NumSubStep;

	const FVector& RopeCenter = Positions[ParticleIdx];

	const FVector& PrevSolveVelocity = PrevSolveVelocities[ParticleIdx];
	const FVector& PrevConstraintSolveVelocity = PrevConstraintSolveVelocities[ParticleIdx];
	const FVector& Velocity = Velocities[ParticleIdx];
	const FVector& Acceleration = Accelerations[ParticleIdx];

	FVector DeltaVelocity = FVector::ZeroVector;

#if 0 // TODO:DiskからSphere形状への変化は後で修正
	for (TPair<TWeakObjectPtr<class UPrimitiveComponent>, FTransform> CollisionPose : CoinBlockerCollisionsPoseMap)
	{
		if (CollisionPose.Key == nullptr)
		{
			continue;
		}

		const FTransform& Pose = CollisionPose.Value;
		// 前フレームで同じコンポーネントがなければ、前フレームのポーズは現フレームのポーズとしておく
		const FTransform* PrevPosePtr = PrevCoinBlockerCollisionsPoseMap.Find(CollisionPose.Key);
		const FTransform& PrevPose = PrevPosePtr == nullptr ? Pose : *PrevPosePtr;
		// イテレーションごとの位置補間は線形補間で行う
		const FVector& CollisionCenter = FMath::Lerp(PrevPose.GetLocation(), Pose.GetLocation(), SubStepAlpha);

		// USphereComponentとUCapsuleComponent以外は今のところ非対応
		const USphereComponent* SphereComp = Cast<const USphereComponent>(CollisionPose.Key.Get());
		if (SphereComp != nullptr)
		{
			const FVector& CollisionCenterProjectedPoint = FVector::PointPlaneProject(CollisionCenter, RopePlane);

			// これは厳密には球への最近接点でない。球がRopeの面の中央部分に接した場合などはRopeの円周部分は
			// 最近接点にならない。球がRopeより十分大きいという前提での近似になっている。
			const FVector& ClosestRopePoint = RopeCenter + (CollisionCenterProjectedPoint - RopeCenter).GetSafeNormal() * RopeRadius;

			//TODO: RopeCenterと球のコリジョンをとらなくていい？

			float SphereRadius = SphereComp->GetScaledSphereRadius();
			if ((ClosestRopePoint - CollisionCenter).SizeSquared() < SphereRadius * SphereRadius)
			{
				const FVector& CollisionVelocity = (Pose.GetLocation() - PrevPose.GetLocation()) / DeltaSeconds;

				FVector ImpactNormal = (RopeCenter - CollisionCenter).GetSafeNormal();
				if (((ClosestRopePoint - CollisionCenter) | (RopeCenter - CollisionCenter)) < 0.0f)
				{
					ImpactNormal *= -1;
				}

				if (CollisionRestitution > KINDA_SMALL_NUMBER)
				{
					SolveRestituionDeltaVelocity(RopeCenter, ClosestRopePoint, ImpactNormal, CollisionRestitution, RestitutionSleepVelocity, PrevSolveVelocity, CollisionVelocity, PrevConstraintSolveVelocity, DeltaVelocity);
				}

				if (CollisionDynamicFriction > KINDA_SMALL_NUMBER)
				{
					SolveDynamicFrictionDeltaVelocity(RopeCenter, ClosestRopePoint, ImpactNormal, CollisionDynamicFriction, PrevSolveVelocity, CollisionVelocity, Acceleration, SubStepDeltaSeconds, DeltaVelocity);
				}
			}
			continue;
		}

		UCapsuleComponent* CapsuleComp = Cast<UCapsuleComponent>(CollisionPose.Key.Get());
		if (CapsuleComp != nullptr)
		{
			FTransform CapsuleTM = Pose;
			// イテレーションごとの位置補間は線形補間で行う
			CapsuleTM.BlendWith(PrevPose, SubStepAlpha);

			float HalfHeight = CapsuleComp->GetScaledCapsuleHalfHeight_WithoutHemisphere();
			const FVector& ZAxis = CapsuleTM.GetUnitAxis(EAxis::Type::Z);
			const FVector& StartPoint = CollisionCenter + ZAxis * HalfHeight;
			const FVector& EndPoint = CollisionCenter - ZAxis * HalfHeight;
			// FMath::PointDistToSegmentSquared()は中でFMath::ClosestPointOnSegment() を呼び出しているので下で2回呼び出すのは無駄なので
			// FMath::ClosestPointOnSegment()を使う
			//float DistSquared = FMath::PointDistToSegmentSquared(Positions[ParticleIdx], StartPoint, EndPoint);
			FVector ClosestPointOnSegment = FMath::ClosestPointOnSegment(RopeCenter, StartPoint, EndPoint);
			const FVector& ClosestPointOnSegmentProjected = FVector::PointPlaneProject(ClosestPointOnSegment, RopePlane);
			const FVector& DeepestPenetratePoint = RopeCenter + (ClosestPointOnSegmentProjected - RopeCenter).GetSafeNormal() * RopeRadius;

			// 厳密にはこのClosestPointOnSegmentとRopeCenterから計算した上のClosestPointOnSegmentは違うが、
			// カプセルがRopeと比較してサイズが大きいものとして近似する。
			ClosestPointOnSegment = FMath::ClosestPointOnSegment(DeepestPenetratePoint, StartPoint, EndPoint);
			float CapsuleRadius = CapsuleComp->GetScaledCapsuleRadius();
			if ((ClosestPointOnSegment - DeepestPenetratePoint).SizeSquared() < CapsuleRadius * CapsuleRadius)
			{
				// インパクトポイントの速度を求める
				const FQuat& RotDiff = Pose.GetRotation() * PrevPose.GetRotation().Inverse();
				const FVector& PrevCenterToDeepestPenetratePoint = RotDiff.UnrotateVector(DeepestPenetratePoint - Pose.GetLocation());
				const FVector& CollisionImpactPointVelocity = (DeepestPenetratePoint - (PrevPose.GetLocation() + PrevCenterToDeepestPenetratePoint)) / DeltaSeconds;

				FVector ImpactNormal = (DeepestPenetratePoint - ClosestPointOnSegment).GetSafeNormal();
				if (((ClosestPointOnSegment - DeepestPenetratePoint) | (ClosestPointOnSegment - RopeCenter)) < 0.0f)
				{
					ImpactNormal *= -1;
				}

				if (CollisionRestitution > KINDA_SMALL_NUMBER)
				{
					SolveRestituionDeltaVelocity(RopeCenter, DeepestPenetratePoint, ImpactNormal, CollisionRestitution, RestitutionSleepVelocity, PrevSolveVelocity, CollisionImpactPointVelocity, PrevConstraintSolveVelocity, DeltaVelocity);
				}

				if (CollisionDynamicFriction > KINDA_SMALL_NUMBER)
				{
					SolveDynamicFrictionDeltaVelocity(RopeCenter, DeepestPenetratePoint, ImpactNormal, CollisionDynamicFriction, PrevSolveVelocity, CollisionImpactPointVelocity, Acceleration, SubStepDeltaSeconds, DeltaVelocity);
				}
			}
			continue;
		}
	}

	Velocities[ParticleIdx] += DeltaVelocity;
#endif
}

void ARopeSimulatorCPU::CalculateOneWallVelocityConstraint(int32 ParticleIdx, const FPlane& WallPlane, float SubStepDeltaSeconds, FVector& InOutDeltaVelocity)
{
	const FVector& RopeCenter = Positions[ParticleIdx];

	const FVector& PrevConstraintSolveVelocity = PrevConstraintSolveVelocities[ParticleIdx];
	const FVector& Velocity = Velocities[ParticleIdx];
	const FVector& Acceleration = Accelerations[ParticleIdx];

	float RestitutionSleepVelocity = 2.0f * FMath::Abs(Gravity) * SubStepDeltaSeconds;

#if 0 // TODO:DiskからSphere形状への変化は後で修正
	FVector DeepestPenetratePoint;
	float DeepestPenetrateDepth;
	bool bCollisioned = CalculateRopeToPlaneCollision(RopePlane, RopeCenter, RopeRadius, WallPlane, DeepestPenetratePoint, DeepestPenetrateDepth);
	if (bCollisioned)
	{
		if (WallRestitution > KINDA_SMALL_NUMBER)
		{
			SolveRestituionDeltaVelocity(RopeCenter, DeepestPenetratePoint, WallPlane.GetNormal(), WallRestitution, RestitutionSleepVelocity, Velocity, FVector::ZeroVector, PrevConstraintSolveVelocity, InOutDeltaVelocity);
		}

		if (WallDynamicFriction > KINDA_SMALL_NUMBER)
		{
			SolveDynamicFrictionDeltaVelocity(RopeCenter, DeepestPenetratePoint, WallPlane.GetNormal(), WallDynamicFriction, Velocity, FVector::ZeroVector, Acceleration, SubStepDeltaSeconds, InOutDeltaVelocity);
		}
	}
#endif
}

void ARopeSimulatorCPU::ApplyWallVelocityConstraint(int32 ParticleIdx, float SubStepDeltaSeconds)
{
	FVector DeltaVelocity = FVector::ZeroVector;

	// MinZ
	CalculateOneWallVelocityConstraint(ParticleIdx, FPlane(WallBox.Max, -FVector::ZAxisVector), SubStepDeltaSeconds, DeltaVelocity);
	// MinX
	CalculateOneWallVelocityConstraint(ParticleIdx, FPlane(WallBox.Min, FVector::XAxisVector), SubStepDeltaSeconds, DeltaVelocity);
	// MaxX
	CalculateOneWallVelocityConstraint(ParticleIdx, FPlane(WallBox.Max, -FVector::XAxisVector), SubStepDeltaSeconds, DeltaVelocity);
	// MinY
	CalculateOneWallVelocityConstraint(ParticleIdx, FPlane(WallBox.Min, FVector::YAxisVector), SubStepDeltaSeconds, DeltaVelocity);
	// MaxY
	CalculateOneWallVelocityConstraint(ParticleIdx, FPlane(WallBox.Max, -FVector::YAxisVector), SubStepDeltaSeconds, DeltaVelocity);

	Velocities[ParticleIdx] += DeltaVelocity;
}

ARopeSimulatorCPU::ARopeSimulatorCPU()
{
	PrimaryActorTick.bCanEverTick = true;

	NiagaraComponent = CreateDefaultSubobject<UNiagaraComponent>(TEXT("NiagaraComponent0"));

	RootComponent = NiagaraComponent;

#if WITH_EDITORONLY_DATA
	SpriteComponent = CreateEditorOnlyDefaultSubobject<UBillboardComponent>(TEXT("Sprite"));
	ArrowComponent = CreateEditorOnlyDefaultSubobject<UArrowComponent>(TEXT("ArrowComponent0"));

	if (!IsRunningCommandlet())
	{
		// Structure to hold one-time initialization
		struct FConstructorStatics
		{
			ConstructorHelpers::FObjectFinderOptional<UTexture2D> SpriteTextureObject;
			FName ID_Effects;
			FText NAME_Effects;
			FConstructorStatics()
				: SpriteTextureObject(TEXT("/Niagara/Icons/S_ParticleSystem"))
				, ID_Effects(TEXT("Effects"))
				, NAME_Effects(NSLOCTEXT("SpriteCategory", "Effects", "Effects"))
			{
			}
		};
		static FConstructorStatics ConstructorStatics;

		if (SpriteComponent)
		{
			SpriteComponent->Sprite = ConstructorStatics.SpriteTextureObject.Get();
			SpriteComponent->SetRelativeScale3D_Direct(FVector(0.5f, 0.5f, 0.5f));
			SpriteComponent->bHiddenInGame = true;
			SpriteComponent->bIsScreenSizeScaled = true;
			SpriteComponent->SpriteInfo.Category = ConstructorStatics.ID_Effects;
			SpriteComponent->SpriteInfo.DisplayName = ConstructorStatics.NAME_Effects;
			SpriteComponent->SetupAttachment(NiagaraComponent);
			SpriteComponent->bReceivesDecals = false;
		}

		if (ArrowComponent)
		{
			ArrowComponent->ArrowColor = FColor(0, 255, 128);

			ArrowComponent->ArrowSize = 1.5f;
			ArrowComponent->bTreatAsASprite = true;
			ArrowComponent->bIsScreenSizeScaled = true;
			ArrowComponent->SpriteInfo.Category = ConstructorStatics.ID_Effects;
			ArrowComponent->SpriteInfo.DisplayName = ConstructorStatics.NAME_Effects;
			ArrowComponent->SetupAttachment(NiagaraComponent);
			ArrowComponent->SetUsingAbsoluteScale(true);
		}
	}
#endif // WITH_EDITORONLY_DATA
}

void ARopeSimulatorCPU::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	// Set Notification Delegate
	if (NiagaraComponent)
	{
		NiagaraComponent->OnSystemFinished.AddUniqueDynamic(this, &ARopeSimulatorCPU::OnNiagaraSystemFinished);
	}
}

void ARopeSimulatorCPU::SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish)
{
	bDestroyOnSystemFinish = bShouldDestroyOnSystemFinish ? 1 : 0;  
};

void ARopeSimulatorCPU::OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent)
{
	if (bDestroyOnSystemFinish)
	{
		SetLifeSpan(0.0001f);
	}
}

#if WITH_EDITOR
bool ARopeSimulatorCPU::GetReferencedContentObjects(TArray<UObject*>& Objects) const
{
	Super::GetReferencedContentObjects(Objects);

	if (UNiagaraSystem* System = NiagaraComponent->GetAsset())
	{
		Objects.Add(System);
	}

	return true;
}

void ARopeSimulatorCPU::ResetInLevel()
{
	if (NiagaraComponent)
	{
		NiagaraComponent->Activate(true);
		NiagaraComponent->ReregisterComponent();
	}
}
#endif // WITH_EDITOR

