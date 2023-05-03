#include "RopeSimulatorCPU.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Texture2D.h"
#include "Components/ArrowComponent.h"
#include "Components/BillboardComponent.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "Async/ParallelFor.h"

void ARopeSimulatorCPU::PreInitializeComponents()
{
	Super::PreInitializeComponents();

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();

	Positions.SetNum(NumParticles);
	PrevPositions.SetNum(NumParticles);
	PrevCurrentIterationPositions.SetNum(NumParticles);
	Velocities.SetNum(NumParticles);
	PrevConstraintSolveVelocities.SetNum(NumParticles);
	PrevSolveVelocities.SetNum(NumParticles);
	InitialOrientations.SetNum(NumParticles);
	Orientations.SetNum(NumParticles);
	Colors.SetNum(NumParticles);
	Accelerations.SetNum(NumParticles);

	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
	{
		PrevCurrentIterationPositions[ParticleIdx] = Positions[ParticleIdx] = FVector(0.0f, 0.0f, -RestLength * ParticleIdx);
		PrevSolveVelocities[ParticleIdx] = PrevConstraintSolveVelocities[ParticleIdx] = Velocities[ParticleIdx] = FVector::ZeroVector;
		Accelerations[ParticleIdx] = InvActorTransform.TransformVectorNoScale(FVector(0.0f, 0.0f, Gravity));

		if (ParticleIdx % 2 == 0)
		{
			Orientations[ParticleIdx] = InitialOrientations[ParticleIdx] = FQuat(FVector::ZAxisVector, PI / 2) * FQuat(FVector::XAxisVector, PI / 2);
		}
		else
		{
			Orientations[ParticleIdx] = InitialOrientations[ParticleIdx] = FQuat(FVector::XAxisVector, PI / 2);
		}

		Colors[ParticleIdx] = FLinearColor::MakeRandomColor();
	}

	// Tick()で設定しても、レベルにNiagaraSystemが最初から配置されていると、初回のスポーンでは配列は初期値を使ってしまい
	//間に合わないのでBeginPlay()でも設定する
	NiagaraComponent->SetNiagaraVariableInt("NumParticles", NumParticles);
	NiagaraComponent->SetNiagaraVariableInt("Shape", 0);
	NiagaraComponent->SetNiagaraVariableVec3("MeshScale", FVector(MeshScale));
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("Positions"), Positions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayQuat(NiagaraComponent, FName("Orientations"), Orientations);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("Colors"), Colors);

	NumThreadParticles = (NumParticles + NumThreads - 1) / NumThreads;
}

void ARopeSimulatorCPU::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		if (EndConstraintActor != nullptr)
		{
			//
			// EndConstraintActorが設定されていれば末端をコンストレイント
			//

			// とりあえずGetActorUpVector()にEndConstraintRadiusだけ離れた位置にコンストレイントさせる形にする
			const FVector& EndConstraintedPosWS = EndConstraintActor->GetActorLocation() + EndConstraintActor->GetActorUpVector() * EndConstraintRadius;

			// テレポートにあたるので、Prev系の変数も初期化し、速度の効果を出さない
			PrevCurrentIterationPositions[NumParticles - 1] = PrevPositions[NumParticles - 1] = Positions[NumParticles - 1] = InvActorTransform.TransformPosition(EndConstraintedPosWS);
			PrevSolveVelocities[NumParticles - 1] = PrevConstraintSolveVelocities[NumParticles - 1] = Velocities[NumParticles - 1] = FVector::ZeroVector;

			//
			// ロープが伸びきる状態になっていたらEndConstraintActorにルートの方向にインパルスを与える。
			//

			// このアクタはEndConstraintActorの物理シミュレーションの影響を遅延なくとりこむために
			// TickGroup = TG_PostPhysicsにしてるので影響は次のフレームからになるがしょうがない。
			const FVector& RootToEnd = (Positions[NumParticles - 1] - Positions[0]);
			float OverLength = RootToEnd.Size() - (NumParticles - 1) * RestLength;
			if (OverLength > 0.0f)
			{
				// RootComponentがUPrimitiveComponent派生のときだけに限定
				UPrimitiveComponent* PrimitiveComponent = Cast<UPrimitiveComponent>(EndConstraintActor->GetRootComponent());
				if (PrimitiveComponent != nullptr)
				{
					const FVector& RootToEndDir = RootToEnd.GetSafeNormal();
					// めりこみに応じた速度変化量を指定する
					// TODO:FrameRateを使っていないし、フレームレートに硬さが依存してしまうが妥協する
					const FVector& AddVelocity = -RootToEndDir * OverLength * ImpulseVelocityPerOver;
					PrimitiveComponent->AddVelocityChangeImpulseAtLocation(AddVelocity, EndConstraintedPosWS);
				}
			}
		}

		// DeltaSecondsの値の変動に関わらず、シミュレーションに使うサブステップタイムは固定とする
		float FixedDeltaSeconds = 1.0f / FrameRate;
		float SubStepDeltaSeconds = FixedDeltaSeconds / NumSubStep;

		UpdateRopeBlockers();

		// Niagara版はParticleAttributeReaderの値がSimStageのイテレーション単位でしか更新されない。
		// そのため、サブステップを複数にしても、Integrate()で落ちた位置をそのサブステップ回でParticleAttributeReaderから参照できない。
		// となると距離コンストレイントのために自由落下が妨げられて遅くなる。
		for (int32 SubStep = 0; SubStep < NumSubStep; ++SubStep)
		{
			// マルチスレッド化するほどの処理でもないのでしない
			for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
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

	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
	{
		// 初期配置がローカル座標でZ軸負方向にチェインがのびていて、そのときに回転が0という前提。
		// 子供と自分の位置差分ベクトルをパーティクルの向きとする。
		if (ParticleIdx == NumParticles - 1)
		{
			// TODO:ダミーパーティクルはやらず親と自分の位置差分にするので親のパーティクルと向きが同じになる
			// という問題は残っている
			const FVector& ParentToSelf = Positions[ParticleIdx] - Positions[ParticleIdx - 1];
			Orientations[ParticleIdx] = FQuat::FindBetween(-FVector::ZAxisVector, ParentToSelf) * InitialOrientations[ParticleIdx];
		}
		else
		{
			const FVector& SelfToChild = Positions[ParticleIdx + 1] - Positions[ParticleIdx];
			Orientations[ParticleIdx] = FQuat::FindBetween(-FVector::ZAxisVector, SelfToChild) * InitialOrientations[ParticleIdx];
		}
	}

	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayQuat(NiagaraComponent, FName("Orientations"), Orientations);
}

void ARopeSimulatorCPU::UpdateRopeBlockers()
{
	TArray<FOverlapResult> Overlaps;
	FCollisionObjectQueryParams ObjectParams(OverlapQueryObjectTypes);
	if (!ObjectParams.IsValid())
	{
		return;
	}

	FCollisionQueryParams Params(SCENE_QUERY_STAT(UpdateRopeBlockers), false);

	GetWorld()->OverlapMultiByObjectType(Overlaps, GetActorLocation() + WallBox.GetCenter(), GetActorQuat(), ObjectParams, FCollisionShape::MakeBox(WallBox.GetExtent()), Params);

	// コリジョンがPrimitiveComponent単位で増減することを想定し、呼ばれるたびにマップを作り直す
	//TODO: PrimitiveComponent内のコリジョンシェイプの増減は想定してない
	PrevRopeBlockerAggGeomMap = RopeBlockerAggGeomMap;
	RopeBlockerAggGeomMap.Reset();

	for (const FOverlapResult& Overlap : Overlaps)
	{
		const UPrimitiveComponent* Primitive = Overlap.GetComponent();
		if (Primitive == nullptr || Primitive->GetBodyInstance() == nullptr || Primitive->GetBodyInstance()->GetBodySetup() == nullptr)
		{
			continue;
		}

		// ローカル座標に変換
		const FTransform& ActorTM = Primitive->GetComponentTransform() * InvActorTransform;

		const FKAggregateGeom& OriginalAggGeom = Primitive->GetBodyInstance()->GetBodySetup()->AggGeom;
		// マップにはFKAggregateGeomのコピーを保存
		RopeBlockerAggGeomMap.Add(Primitive, Primitive->GetBodyInstance()->GetBodySetup()->AggGeom);

		// FKAggregateGeomのコピーコンストラクトを少なくしたいのでマップにAddしてから編集する
		FKAggregateGeom* CacheAggGeom = RopeBlockerAggGeomMap.Find(Primitive);

		// キャッシュするFKAggregateGeomにはActorのScaleを適用しておく
		for (int32 i = 0; i < OriginalAggGeom.SphereElems.Num(); ++i)
		{
			CacheAggGeom->SphereElems[i] = OriginalAggGeom.SphereElems[i].GetFinalScaled(FVector::OneVector, ActorTM);
		}

		for (int32 i = 0; i < OriginalAggGeom.BoxElems.Num(); ++i)
		{
			CacheAggGeom->BoxElems[i] = OriginalAggGeom.BoxElems[i].GetFinalScaled(FVector::OneVector, ActorTM);
		}

		for (int32 i = 0; i < OriginalAggGeom.SphylElems.Num(); ++i)
		{
			CacheAggGeom->SphylElems[i] = OriginalAggGeom.SphylElems[i].GetFinalScaled(FVector::OneVector, ActorTM);
		}
	}
}

void ARopeSimulatorCPU::Integrate(int32 ParticleIdx, float SubStepDeltaSeconds)
{
	PrevPositions[ParticleIdx] = Positions[ParticleIdx];

	// 0番目は固定点
	if (ParticleIdx == 0)
	{
		return;
	}

	// 末端が固定されているとき
	if ((EndConstraintActor != nullptr) && (ParticleIdx == NumParticles - 1))
	{
		return;
	}

	Velocities[ParticleIdx] += Accelerations[ParticleIdx] * SubStepDeltaSeconds;
	Positions[ParticleIdx] += Velocities[ParticleIdx] * SubStepDeltaSeconds;
}

void ARopeSimulatorCPU::SolvePositionConstraint(int32 InFrameExeCount)
{
	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
	{
		PrevCurrentIterationPositions[ParticleIdx] = Positions[ParticleIdx];
	}

	ParallelFor(NumThreads,
		[this, InFrameExeCount](int32 ThreadIndex)
		{
			for (int32 ParticleIdx = NumThreadParticles * ThreadIndex; ParticleIdx < NumThreadParticles * (ThreadIndex + 1) && ParticleIdx < NumParticles; ++ParticleIdx)
			{
				// 0番目は固定点
				if (ParticleIdx == 0)
				{
					continue;
				}

				// 末端が固定されているとき
				if ((EndConstraintActor != nullptr) && (ParticleIdx == NumParticles - 1))
				{
					continue;
				}

				ApplyDistanceConstraint(ParticleIdx);
				ApplyRopeBlockersCollisionConstraint(ParticleIdx, InFrameExeCount);
				ApplyWallCollisionConstraint(ParticleIdx);
			}
		}
	);
}

void ARopeSimulatorCPU::ApplyDistanceConstraint(int32 ParticleIdx)
{
	FVector Move = FVector::ZeroVector;

	// Parent-Self
	{
		const FVector& ParentToSelf = PrevCurrentIterationPositions[ParticleIdx] - PrevCurrentIterationPositions[ParticleIdx - 1];
		const FVector& ParentToSelfDir = ParentToSelf.GetSafeNormal();
		float DiffLength = ParentToSelf.Size() - RestLength;

		Move -= ParentToSelfDir * DiffLength;
	}

	// Self-Child
	if (ParticleIdx < NumParticles - 1) // 末端はChildがない
	{
		const FVector& ChildToSelf = PrevCurrentIterationPositions[ParticleIdx] - PrevCurrentIterationPositions[ParticleIdx + 1];
		const FVector& ChildToSelfDir = ChildToSelf.GetSafeNormal();
		float DiffLength = ChildToSelf.Size() - RestLength;

		Move -= ChildToSelfDir * DiffLength;
	}

	Positions[ParticleIdx] += Move * 0.5f; // 0.5fで平均している
}

void ARopeSimulatorCPU::SolveVelocity(float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount)
{
	// 確定したPositionsから一旦Velocitiesを決定する
	float InvSubStepDeltaSeconds = 1.0f / SubStepDeltaSeconds;
	float InvSquareSubStepDeltaSeconds = InvSubStepDeltaSeconds * InvSubStepDeltaSeconds;
	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ++ParticleIdx)
	{
		// 位置コンストレイントで受ける力の総和を計算。コリジョンの押し返される反作用力と距離コンストレイントの実効力。
		// Accelerations変数を外力の加速度として使うだけでなく、位置コンストレイントで受ける力の総和として再利用する。
		Accelerations[ParticleIdx] = (Positions[ParticleIdx] - PrevCurrentIterationPositions[ParticleIdx]) * InvSquareSubStepDeltaSeconds;

		PrevConstraintSolveVelocities[ParticleIdx] = Velocities[ParticleIdx]; // 位置コンストレイントと静止摩擦で位置が修正される前の速度。反発の計算に使う
		Velocities[ParticleIdx] = (Positions[ParticleIdx] - PrevPositions[ParticleIdx]) * InvSubStepDeltaSeconds;
		PrevSolveVelocities[ParticleIdx] = Velocities[ParticleIdx]; // 位置コンストレイントと静止摩擦を反映した速度
	}

	if (CollisionRestitution > KINDA_SMALL_NUMBER || CollisionDynamicFriction > KINDA_SMALL_NUMBER || WallRestitution > KINDA_SMALL_NUMBER || WallDynamicFriction > KINDA_SMALL_NUMBER)
	{
		ParallelFor(NumThreads,
			[DeltaSeconds, SubStepDeltaSeconds, SubStepCount, this](int32 ThreadIndex)
			{
				for (int32 ParticleIdx = NumThreadParticles * ThreadIndex; ParticleIdx < NumThreadParticles * (ThreadIndex + 1) && ParticleIdx < NumParticles; ++ParticleIdx)
				{
					// 0番目は固定点
					if (ParticleIdx == 0)
					{
						continue;
					}

					// 末端が固定されているとき
					if ((EndConstraintActor != nullptr) && (ParticleIdx == NumParticles - 1))
					{
						continue;
					}

					if (CollisionRestitution > KINDA_SMALL_NUMBER || CollisionDynamicFriction > KINDA_SMALL_NUMBER)
					{
						ApplyRopeBlockersVelocityConstraint(ParticleIdx, DeltaSeconds, SubStepDeltaSeconds, SubStepCount);
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

void ARopeSimulatorCPU::ApplyRopeBlockersCollisionConstraint(int32 ParticleIdx, int32 InFrameExeCount)
{
	float FrameExeAlpha = (InFrameExeCount + 1) / (NumSubStep * NumIteration);

	const FVector& RopeCenter = Positions[ParticleIdx];

	// TODO:コリジョンについても、NeighborGridに入ってるものだけを処理するようにすればコストは減らせる。まあ数がパーティクルほど多くないし複数のセルにまたがるものが多いだろうからやるのも微妙だが
	// そもそも物理アセット、あるいはプレイヤーのカプセルがこのアクタと交差してなければコリジョン計算の必要すらない。
	// TODO:現フレームの現イテレーションでのコリジョンの位置と向きを前フレームのポーズでのコリジョン位置と現フレームのポーズでのコリジョン位置から
	// 補間で求めているが、それをパーティクルごとにやるのはもったいない。キャッシュしたいね。キャッシュもイテレーションごとにやる必要出るが。
	for (TPair<TWeakObjectPtr<const UPrimitiveComponent>, FKAggregateGeom> Pair : RopeBlockerAggGeomMap)
	{
		if (Pair.Key == nullptr)
		{
			continue;
		}

		const FKAggregateGeom& AggGeom = Pair.Value;
		const FKAggregateGeom* PrevAggGeomPair = PrevRopeBlockerAggGeomMap.Find(Pair.Key);
		const FKAggregateGeom& PrevAggGeom = PrevAggGeomPair == nullptr ? AggGeom : *PrevAggGeomPair;

		for (int32 i = 0; i < AggGeom.SphereElems.Num(); ++i)
		{
			// FKSphereElem::GetClosestPointAndNormalを参考にしている
			const FKSphereElem& SphereElem = AggGeom.SphereElems[i];
			const FKSphereElem& PrevSphereElem = PrevAggGeom.SphereElems[i];

			// イテレーションごとの位置補間は線形補間で行う
			const FVector& CollisionCenter = FMath::Lerp(PrevSphereElem.Center, SphereElem.Center, FrameExeAlpha);

			float LimitDistance = RopeRadius + SphereElem.Radius;
			if ((RopeCenter - CollisionCenter).SizeSquared() < LimitDistance * LimitDistance)
			{
				Positions[ParticleIdx] += (RopeCenter - CollisionCenter).GetSafeNormal() * (LimitDistance - (RopeCenter - CollisionCenter).Size()) * CollisionProjectionAlpha;
			}
		}

		for (int32 i = 0; i < AggGeom.BoxElems.Num(); ++i)
		{
			// FKBoxElem::GetClosestPointAndNormalを参考にしている
			FKBoxElem BoxElem = AggGeom.BoxElems[i];

			// 球とBoxの当たり判定だと球がBoxの中に入ったかどうかで分岐して面倒なので、Boxを球の半径だけ大きくして点とBoxの当たり判定にする
			BoxElem.X += RopeRadius * 2;
			BoxElem.Y += RopeRadius * 2;
			BoxElem.Z += RopeRadius * 2;

			// Boxとの当たり判定ではBoxのローカル座標系で計算したほうがいい
			FTransform BoxTM = PrevAggGeom.BoxElems[i].GetTransform();
			// イテレーションごとの位置補間は線形補間で行う
			BoxTM.BlendWith(BoxElem.GetTransform(), FrameExeAlpha);
			const FVector& BoxLocalPosition = BoxTM.InverseTransformPositionNoScale(RopeCenter);

			const float HalfX = BoxElem.X * 0.5f;
			const float HalfY = BoxElem.Y * 0.5f;
			const float HalfZ = BoxElem.Z * 0.5f;

			bool bIsInside = BoxLocalPosition.X > -HalfX && BoxLocalPosition.X < HalfX && BoxLocalPosition.Y > -HalfY && BoxLocalPosition.Y < HalfY && BoxLocalPosition.Z > -HalfZ && BoxLocalPosition.Z < HalfZ;
			if (bIsInside)
			{
				float DistToX = HalfX - FMath::Abs(BoxLocalPosition.X);
				float DistToY = HalfY - FMath::Abs(BoxLocalPosition.Y);
				float DistToZ = HalfZ - FMath::Abs(BoxLocalPosition.Z);

				FVector ClosestLocalPosition = BoxLocalPosition;
				if (DistToX < DistToY)
				{
					if (DistToX < DistToZ)
					{
						ClosestLocalPosition.X = BoxLocalPosition.X > 0.0f ? HalfX : -HalfX;
					}
					else
					{
						ClosestLocalPosition.Z = BoxLocalPosition.Z > 0.0f ? HalfZ : -HalfZ;
					}
				}
				else // DistToY <= DistToX
				{
					if (DistToY < DistToZ)
					{
						ClosestLocalPosition.Y = BoxLocalPosition.Y > 0.0f ? HalfY : -HalfY;
					}
					else
					{
						ClosestLocalPosition.Z = BoxLocalPosition.Z > 0.0f ? HalfZ : -HalfZ;
					}
				}

				Positions[ParticleIdx] += (BoxTM.TransformPositionNoScale(ClosestLocalPosition) - RopeCenter) * CollisionProjectionAlpha;
			}
		}

		for (int32 i = 0; i < AggGeom.SphylElems.Num(); ++i)
		{
			// FKSphylElem::GetClosestPointAndNormalを参考にしている
			const FKSphylElem& SphylElem = AggGeom.SphylElems[i];
			const FKSphylElem& PrevSphylElem = PrevAggGeom.SphylElems[i];

			// イテレーションごとの位置補間は線形補間で行う
			const FVector& CollisionCenter = FMath::Lerp(PrevSphylElem.Center, SphylElem.Center, FrameExeAlpha);

			FTransform CapsuleTM = PrevSphylElem.GetTransform();
			// イテレーションごとの位置補間は線形補間で行う
			CapsuleTM.BlendWith(SphylElem.GetTransform(), FrameExeAlpha);

			const FVector& StartPoint = CollisionCenter + CapsuleTM.GetUnitAxis(EAxis::Type::Z) * SphylElem.Length * 0.5f;
			const FVector& EndPoint = CollisionCenter + CapsuleTM.GetUnitAxis(EAxis::Type::Z) * SphylElem.Length * -0.5f;
			// FMath::PointDistToSegmentSquared()は中でFMath::ClosestPointOnSegment() を呼び出しているので下で2回呼び出すのは無駄なので
			// FMath::ClosestPointOnSegment()を使う
			//float DistSquared = FMath::PointDistToSegmentSquared(RopeCenter, StartPoint, EndPoint);
			const FVector& ClosestPoint = FMath::ClosestPointOnSegment(RopeCenter, StartPoint, EndPoint);
			float DistSquared = (RopeCenter - ClosestPoint).SizeSquared();

			float LimitDistance = RopeRadius + SphylElem.Radius;
			if (DistSquared < LimitDistance * LimitDistance)
			{
				Positions[ParticleIdx] += (RopeCenter - ClosestPoint).GetSafeNormal() * (LimitDistance - (RopeCenter - ClosestPoint).Size()) * CollisionProjectionAlpha;
			}
		}
	}
}

void ARopeSimulatorCPU::ApplyWallCollisionConstraint(int32 ParticleIdx)
{
	const FVector& RopeCenter = Positions[ParticleIdx];

	FVector DeltaPos = FVector::ZeroVector;

	// ブレークポイントをはりやすいようにFMath::Max()でなくifにしている
	if (((RopeCenter.Z + RopeRadius) - WallBox.Max.Z) > 0.0f)
	{
		DeltaPos += ((RopeCenter.Z + RopeRadius) - WallBox.Max.Z) * FVector(0.0f, 0.0f, -1.0f) * WallProjectionAlpha;
	}

	if ((WallBox.Min.Z - (RopeCenter.Z - RopeRadius)) > 0.0f)
	{
		DeltaPos += (WallBox.Min.Z - (RopeCenter.Z - RopeRadius)) * FVector(0.0f, 0.0f, 1.0f) * WallProjectionAlpha;
	}

	if ((WallBox.Min.X - (RopeCenter.X - RopeRadius)) > 0.0f)
	{
		DeltaPos += (WallBox.Min.X - (RopeCenter.X - RopeRadius)) * FVector(1.0f, 0.0f, 0.0f) * WallProjectionAlpha;
	}

	if (((RopeCenter.X + RopeRadius) - WallBox.Max.X) > 0.0f)
	{
		DeltaPos += ((RopeCenter.X + RopeRadius) - WallBox.Max.X) * FVector(-1.0f, 0.0f, 0.0f) * WallProjectionAlpha;
	}

	if ((WallBox.Min.Y - (RopeCenter.Y - RopeRadius)) > 0.0f)
	{
		DeltaPos += (WallBox.Min.Y - (RopeCenter.Y - RopeRadius)) * FVector(0.0f, 1.0f, 0.0f) * WallProjectionAlpha;
	}

	if (((RopeCenter.Y + RopeRadius) - WallBox.Max.Y) > 0.0f)
	{
		DeltaPos += ((RopeCenter.Y + RopeRadius) - WallBox.Max.Y) * FVector(0.0f, -1.0f, 0.0f) * WallProjectionAlpha;
	}

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

void ARopeSimulatorCPU::ApplyRopeBlockersVelocityConstraint(int32 ParticleIdx, float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount)
{
	float RestitutionSleepVelocity = 2.0f * FMath::Abs(Gravity) * SubStepDeltaSeconds;
	float SubStepAlpha = (SubStepCount + 1) / NumSubStep;

	const FVector& RopeCenter = Positions[ParticleIdx];

	for (TPair<TWeakObjectPtr<const UPrimitiveComponent>, FKAggregateGeom> Pair : RopeBlockerAggGeomMap)
	{
		if (Pair.Key == nullptr)
		{
			continue;
		}

		const FKAggregateGeom& AggGeom = Pair.Value;
		const FKAggregateGeom* PrevAggGeomPair = PrevRopeBlockerAggGeomMap.Find(Pair.Key);
		const FKAggregateGeom& PrevAggGeom = PrevAggGeomPair == nullptr ? AggGeom : *PrevAggGeomPair;

		for (int32 j = 0; j <AggGeom.SphereElems.Num(); ++j)
		{
			// FKSphereElem::GetClosestPointAndNormalを参考にしている
			const FKSphereElem& SphereElem = AggGeom.SphereElems[j];
			const FKSphereElem& PrevSphereElem = PrevAggGeom.SphereElems[j];

			// イテレーションごとの位置補間は線形補間で行う
			const FVector& CollisionCenter = FMath::Lerp(PrevSphereElem.Center, SphereElem.Center, SubStepAlpha);
			const FVector& CollisionVelocity = (SphereElem.Center - PrevSphereElem.Center) / DeltaSeconds;

			float LimitDistance = RopeRadius + SphereElem.Radius;
			if ((RopeCenter - CollisionCenter).SizeSquared() < LimitDistance * LimitDistance)
			{
				const FVector& PrevSolveRelativeVelocity = PrevSolveVelocities[ParticleIdx] - CollisionVelocity;
				const FVector& ImpactNormal = (RopeCenter - CollisionCenter).GetSafeNormal();

				float PrevSolveRelativeVelocityNormal = PrevSolveRelativeVelocity | ImpactNormal;
				if (CollisionRestitution > KINDA_SMALL_NUMBER && FMath::Abs(PrevSolveRelativeVelocityNormal) > RestitutionSleepVelocity)
				{
					const FVector& PrevConstraintSolveRelativeVelocity = PrevConstraintSolveVelocities[ParticleIdx] - CollisionVelocity;
					float PrevConstraintSolveRelativeVelocityNormal = PrevConstraintSolveRelativeVelocity | ImpactNormal;
					Velocities[ParticleIdx] += ImpactNormal * (-PrevSolveRelativeVelocityNormal + FMath::Max(-CollisionRestitution * PrevConstraintSolveRelativeVelocityNormal, 0.0f));
				}

				if (CollisionDynamicFriction > KINDA_SMALL_NUMBER)
				{
					const FVector& PrevSolveRelativeVelocityTangent = PrevSolveRelativeVelocity - PrevSolveRelativeVelocityNormal * ImpactNormal;
					float ForceNormalLen = FMath::Abs(Accelerations[ParticleIdx] | ImpactNormal);
					float PrevSolveRelativeVelocityTangentLen = PrevSolveRelativeVelocityTangent.Size();
					Velocities[ParticleIdx] -= PrevSolveRelativeVelocityTangent / FMath::Max(PrevSolveRelativeVelocityTangentLen, KINDA_SMALL_NUMBER) * FMath::Min(SubStepDeltaSeconds * CollisionDynamicFriction * ForceNormalLen, PrevSolveRelativeVelocityTangentLen);
				}
			}
		}

		for (int32 j = 0; j <AggGeom.BoxElems.Num(); ++j)
		{
			// FKBoxElem::GetClosestPointAndNormalを参考にしている
			FKBoxElem BoxElem = AggGeom.BoxElems[j];
			const FKBoxElem& PrevBoxElem = PrevAggGeom.BoxElems[j];

			// 球とBoxの当たり判定だと球がBoxの中に入ったかどうかで分岐して面倒なので、Boxを球の半径だけ大きくして点とBoxの当たり判定にする
			BoxElem.X += RopeRadius * 2;
			BoxElem.Y += RopeRadius * 2;
			BoxElem.Z += RopeRadius * 2;

			// Boxとの当たり判定ではワールド基準でなくBoxのローカル座標系で計算したほうがいい
			FTransform BoxTM = PrevBoxElem.GetTransform();
			// イテレーションごとの位置補間は線形補間で行う
			BoxTM.BlendWith(BoxElem.GetTransform(), SubStepAlpha);
			const FVector& BoxLocalPosition = BoxTM.InverseTransformPositionNoScale(RopeCenter);

			const float HalfX = BoxElem.X * 0.5f;
			const float HalfY = BoxElem.Y * 0.5f;
			const float HalfZ = BoxElem.Z * 0.5f;

			bool bIsInside = BoxLocalPosition.X > -HalfX && BoxLocalPosition.X < HalfX && BoxLocalPosition.Y > -HalfY && BoxLocalPosition.Y < HalfY && BoxLocalPosition.Z > -HalfZ && BoxLocalPosition.Z < HalfZ;
			if (bIsInside)
			{
				float DistToX = HalfX - FMath::Abs(BoxLocalPosition.X);
				float DistToY = HalfY - FMath::Abs(BoxLocalPosition.Y);
				float DistToZ = HalfZ - FMath::Abs(BoxLocalPosition.Z);

				FVector ClosestLocalPosition = BoxLocalPosition;
				FVector ImpactNormal = FVector::ZeroVector;
				if (DistToX < DistToY)
				{
					if (DistToX < DistToZ)
					{
						float Sign = BoxLocalPosition.X > 0.0f ? 1.0f : -1.0f;
						ClosestLocalPosition.X = HalfX * Sign;
						ImpactNormal = FVector::XAxisVector * Sign;
					}
					else
					{
						float Sign = BoxLocalPosition.Z > 0.0f ? 1.0f : -1.0f;
						ClosestLocalPosition.Z = HalfZ * Sign;
						ImpactNormal = FVector::ZAxisVector * Sign;
					}
				}
				else // DistToY <= DistToX
				{
					if (DistToY < DistToZ)
					{
						float Sign = BoxLocalPosition.Y > 0.0f ? 1.0f : -1.0f;
						ClosestLocalPosition.Y = HalfY * Sign;
						ImpactNormal = FVector::YAxisVector * Sign;
					}
					else
					{
						float Sign = BoxLocalPosition.Z > 0.0f ? 1.0f : -1.0f;
						ClosestLocalPosition.Z = HalfZ * Sign;
						ImpactNormal = FVector::ZAxisVector * Sign;
					}
				}

				// インパクトポイントの速度を求める
				const FVector& CenterToClosestPos = BoxTM.GetRotation().RotateVector(ClosestLocalPosition);
				const FQuat& RotDiff = BoxElem.GetTransform().GetRotation() * PrevBoxElem.GetTransform().GetRotation().Inverse();
				const FVector& PrevCenterToClosestPos = RotDiff.UnrotateVector(CenterToClosestPos);
				const FVector& CollisionImpactPointVelocity = ((BoxElem.Center + CenterToClosestPos) - (PrevBoxElem.Center + PrevCenterToClosestPos)) / DeltaSeconds;

				const FVector& PrevSolveRelativeVelocity = PrevSolveVelocities[ParticleIdx] - CollisionImpactPointVelocity;
				ImpactNormal = BoxTM.TransformVectorNoScale(ImpactNormal);

				float PrevSolveRelativeVelocityNormal = PrevSolveRelativeVelocity | ImpactNormal;
				if (CollisionRestitution > KINDA_SMALL_NUMBER && FMath::Abs(PrevSolveRelativeVelocityNormal) > RestitutionSleepVelocity)
				{
					const FVector& PrevConstraintSolveRelativeVelocity = PrevConstraintSolveVelocities[ParticleIdx] - CollisionImpactPointVelocity;
					float PrevConstraintSolveRelativeVelocityNormal = PrevConstraintSolveRelativeVelocity | ImpactNormal;
					Velocities[ParticleIdx] += ImpactNormal * (-PrevSolveRelativeVelocityNormal + FMath::Max(-CollisionRestitution * PrevConstraintSolveRelativeVelocityNormal, 0.0f));
				}

				if (CollisionDynamicFriction > KINDA_SMALL_NUMBER)
				{
					const FVector& PrevSolveRelativeVelocityTangent = PrevSolveRelativeVelocity - PrevSolveRelativeVelocityNormal * ImpactNormal;
					float ForceNormalLen = FMath::Abs(Accelerations[ParticleIdx] | ImpactNormal);
					float PrevSolveRelativeVelocityTangentLen = PrevSolveRelativeVelocityTangent.Size();
					Velocities[ParticleIdx] -= PrevSolveRelativeVelocityTangent / FMath::Max(PrevSolveRelativeVelocityTangentLen, KINDA_SMALL_NUMBER) * FMath::Min(SubStepDeltaSeconds * CollisionDynamicFriction * ForceNormalLen, PrevSolveRelativeVelocityTangentLen);
				}
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

			const FVector& ZAxis = CapsuleTM.GetUnitAxis(EAxis::Type::Z);
			const FVector& StartPoint = CollisionCenter + ZAxis * SphylElem.Length * 0.5f;
			const FVector& EndPoint = CollisionCenter + ZAxis * SphylElem.Length * -0.5f;
			// FMath::PointDistToSegmentSquared()は中でFMath::ClosestPointOnSegment() を呼び出しているので下で2回呼び出すのは無駄なので
			// FMath::ClosestPointOnSegment()を使う
			//float DistSquared = FMath::PointDistToSegmentSquared(RopeCenter, StartPoint, EndPoint);
			const FVector& ClosestPoint = FMath::ClosestPointOnSegment(RopeCenter, StartPoint, EndPoint);
			float DistSquared = (RopeCenter - ClosestPoint).SizeSquared();

			float LimitDistance = RopeRadius + SphylElem.Radius;
			if (DistSquared < LimitDistance * LimitDistance)
			{
				// インパクトポイントの速度を求める
				const FVector& ClosestLocalPosition = CapsuleTM.InverseTransformPosition(ClosestPoint);
				const FVector& CenterToClosestPos = CapsuleTM.GetRotation().RotateVector(ClosestLocalPosition);
				const FQuat& RotDiff = SphylElem.GetTransform().GetRotation() * PrevSphylElem.GetTransform().GetRotation().Inverse();
				const FVector& PrevCenterToClosestPos = RotDiff.UnrotateVector(CenterToClosestPos);
				const FVector& CollisionImpactPointVelocity = ((SphylElem.Center + CenterToClosestPos) - (PrevSphylElem.Center + PrevCenterToClosestPos)) / DeltaSeconds;

				const FVector& PrevSolveRelativeVelocity = PrevSolveVelocities[ParticleIdx] - CollisionImpactPointVelocity;

				const FVector& ImpactNormal = (RopeCenter - ClosestPoint).GetSafeNormal();

				float PrevSolveRelativeVelocityNormal = PrevSolveRelativeVelocity | ImpactNormal;
				if (CollisionRestitution > KINDA_SMALL_NUMBER && FMath::Abs(PrevSolveRelativeVelocityNormal) > RestitutionSleepVelocity)
				{
					const FVector& PrevConstraintSolveRelativeVelocity = PrevConstraintSolveVelocities[ParticleIdx] - CollisionImpactPointVelocity;
					float PrevConstraintSolveRelativeVelocityNormal = PrevConstraintSolveRelativeVelocity | ImpactNormal;
					Velocities[ParticleIdx] += ImpactNormal * (-PrevSolveRelativeVelocityNormal + FMath::Max(-CollisionRestitution * PrevConstraintSolveRelativeVelocityNormal, 0.0f));
				}

				if (CollisionDynamicFriction > KINDA_SMALL_NUMBER)
				{
					const FVector& PrevSolveRelativeVelocityTangent = PrevSolveRelativeVelocity - PrevSolveRelativeVelocityNormal * ImpactNormal;
					float ForceNormalLen = FMath::Abs(Accelerations[ParticleIdx] | ImpactNormal);
					float PrevSolveRelativeVelocityTangentLen = PrevSolveRelativeVelocityTangent.Size();
					Velocities[ParticleIdx] -= PrevSolveRelativeVelocityTangent / FMath::Max(PrevSolveRelativeVelocityTangentLen, KINDA_SMALL_NUMBER) * FMath::Min(SubStepDeltaSeconds * CollisionDynamicFriction * ForceNormalLen, PrevSolveRelativeVelocityTangentLen);
				}
			}
		}
	}
}

namespace
{
	FVector SolveWallRestituionDeltaVelocity(const FVector& WallNormal, float WallRestitution, float RestitutionSleepVelocity, const FVector& Velocity, const FVector& PrevConstraintSolveVelocity)
	{
		FVector DeltaVelocity = FVector::ZeroVector;

		float RelativeVelocityNormal = Velocity | WallNormal;
		if (FMath::Abs(RelativeVelocityNormal) > RestitutionSleepVelocity)
		{
			float PrevConstraintSolveRelativeVelocityNormal = PrevConstraintSolveVelocity | WallNormal;
			DeltaVelocity = WallNormal * (-RelativeVelocityNormal + FMath::Max(-WallRestitution * PrevConstraintSolveRelativeVelocityNormal, 0.0f));
		}

		return DeltaVelocity;
	}

	FVector SolveWallDynamicFrictionDeltaVelocity(const FVector& WallNormal, float WallDynamicFriction, const FVector& Velocity, const FVector& Acceleration, float SubStepDeltaSeconds)
	{
		float RelativeVelocityNormal = Velocity | WallNormal;
		const FVector& RelativeVelocityTangent = Velocity - RelativeVelocityNormal * WallNormal;
		float ForceNormalLen = FMath::Abs(Acceleration | WallNormal);
		float RelativeVelocityTangentLen = RelativeVelocityTangent.Size();
		return -RelativeVelocityTangent / FMath::Max(RelativeVelocityTangentLen, KINDA_SMALL_NUMBER) * FMath::Min(SubStepDeltaSeconds * WallDynamicFriction * ForceNormalLen, RelativeVelocityTangentLen);
	}
}

void ARopeSimulatorCPU::ApplyWallVelocityConstraint(int32 ParticleIdx, float SubStepDeltaSeconds)
{
	const FVector& RopeCenter = Positions[ParticleIdx];
	const FVector& PrevConstraintSolveVelocity = PrevConstraintSolveVelocities[ParticleIdx];
	const FVector& Velocity = Velocities[ParticleIdx];
	const FVector& Acceleration = Accelerations[ParticleIdx];

	float RestitutionSleepVelocity = 2.0f * FMath::Abs(Gravity) * SubStepDeltaSeconds;

	// TODO:もともと壁にもぐっている状態で速度が0のものは反発できないからどうすべきかということに解答が出せていない
	FVector DeltaVelocity = FVector::ZeroVector;
	if (((RopeCenter.Z + RopeRadius) - WallBox.Max.Z) > 0.0f)
	{
		const FVector& Normal = FVector(0.0f, 0.0f, -1.0f);
		if (WallRestitution > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallRestituionDeltaVelocity(Normal, WallRestitution, RestitutionSleepVelocity, Velocity, PrevConstraintSolveVelocity);
		}

		if (WallDynamicFriction > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallDynamicFrictionDeltaVelocity(Normal, WallDynamicFriction, Velocity, Acceleration, SubStepDeltaSeconds);
		}
	}

	if ((WallBox.Min.Z - (RopeCenter.Z - RopeRadius)) > 0.0f)
	{
		const FVector& Normal = FVector(0.0f, 0.0f, 1.0f);
		if (WallRestitution > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallRestituionDeltaVelocity(Normal, WallRestitution, RestitutionSleepVelocity, Velocity, PrevConstraintSolveVelocity);
		}

		if (WallDynamicFriction > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallDynamicFrictionDeltaVelocity(Normal, WallDynamicFriction, Velocity, Acceleration, SubStepDeltaSeconds);
		}
	}

	if ((WallBox.Min.X - (RopeCenter.X - RopeRadius)) > 0.0f)
	{
		const FVector& Normal = FVector(1.0f, 0.0f, 0.0f);
		if (WallRestitution > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallRestituionDeltaVelocity(Normal, WallRestitution, RestitutionSleepVelocity, Velocity, PrevConstraintSolveVelocity);
		}

		if (WallDynamicFriction > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallDynamicFrictionDeltaVelocity(Normal, WallDynamicFriction, Velocity, Acceleration, SubStepDeltaSeconds);
		}
	}

	if (((RopeCenter.X + RopeRadius) - WallBox.Max.X) > 0.0f)
	{
		const FVector& Normal = FVector(-1.0f, 0.0f, 0.0f);
		if (WallRestitution > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallRestituionDeltaVelocity(Normal, WallRestitution, RestitutionSleepVelocity, Velocity, PrevConstraintSolveVelocity);
		}

		if (WallDynamicFriction > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallDynamicFrictionDeltaVelocity(Normal, WallDynamicFriction, Velocity, Acceleration, SubStepDeltaSeconds);
		}
	}

	if ((WallBox.Min.Y - (RopeCenter.Y - RopeRadius)) > 0.0f)
	{
		const FVector& Normal = FVector(0.0f, 1.0f, 0.0f);
		if (WallRestitution > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallRestituionDeltaVelocity(Normal, WallRestitution, RestitutionSleepVelocity, Velocity, PrevConstraintSolveVelocity);
		}

		if (WallDynamicFriction > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallDynamicFrictionDeltaVelocity(Normal, WallDynamicFriction, Velocity, Acceleration, SubStepDeltaSeconds);
		}
	}

	if (((RopeCenter.Y + RopeRadius) - WallBox.Max.Y) > 0.0f)
	{
		const FVector& Normal = FVector(0.0f, -1.0f, 0.0f);
		if (WallRestitution > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallRestituionDeltaVelocity(Normal, WallRestitution, RestitutionSleepVelocity, Velocity, PrevConstraintSolveVelocity);
		}

		if (WallDynamicFriction > KINDA_SMALL_NUMBER)
		{
			DeltaVelocity += SolveWallDynamicFrictionDeltaVelocity(Normal, WallDynamicFriction, Velocity, Acceleration, SubStepDeltaSeconds);
		}
	}

	Velocities[ParticleIdx] += DeltaVelocity;
}

ARopeSimulatorCPU::ARopeSimulatorCPU()
{
	PrimaryActorTick.bCanEverTick = true;
	// EndConstraintActorの物理シミュレーションが終わった後にその位置に応じてシミュレーションしたいので
	PrimaryActorTick.TickGroup = TG_PostPhysics;

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

