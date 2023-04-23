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

	// キャラクターが増減したりコリジョンコンポーネントが増減することを想定し、呼ばれるたびに配列を作り直す
	PrevRopeBlockerCollisionsPoseMap = RopeBlockerCollisionsPoseMap;
	RopeBlockerCollisionsPoseMap.Reset();

	for (const FOverlapResult& Overlap : Overlaps)
	{
		if (Overlap.Component != nullptr)
		{
			RopeBlockerCollisionsPoseMap.Add(Overlap.Component, Overlap.Component->GetComponentTransform() * InvActorTransform);
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

	// bConstraintEndPosition==trueのときは末端も固定
	if (bConstraintEndPosition && (ParticleIdx == NumParticles - 1))
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

				// bConstraintEndPosition==trueのときは末端も固定
				if (bConstraintEndPosition && (ParticleIdx == NumParticles - 1))
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

					// bConstraintEndPosition==trueのときは末端も固定
					if (bConstraintEndPosition && (ParticleIdx == NumParticles - 1))
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

	for (TPair<TWeakObjectPtr<class UPrimitiveComponent>, FTransform> CollisionPose : RopeBlockerCollisionsPoseMap)
	{
		if (CollisionPose.Key == nullptr)
		{
			continue;
		}

		const FTransform& Pose = CollisionPose.Value;
		// 前フレームで同じコンポーネントがなければ、前フレームのポーズは現フレームのポーズとしておく
		const FTransform* PrevPosePtr = PrevRopeBlockerCollisionsPoseMap.Find(CollisionPose.Key);
		const FTransform& PrevPose = PrevPosePtr == nullptr ? Pose : *PrevPosePtr;
		// イテレーションごとの位置補間は線形補間で行う
		const FVector& CollisionCenter = FMath::Lerp(PrevPose.GetLocation(), Pose.GetLocation(), FrameExeAlpha);

		// USphereComponentとUCapsuleComponent以外は今のところ非対応
		const USphereComponent* SphereComp = Cast<const USphereComponent>(CollisionPose.Key.Get());
		if (SphereComp != nullptr)
		{
			float LimitDistance = RopeRadius + SphereComp->GetScaledSphereRadius();
			if ((RopeCenter - CollisionCenter).SizeSquared() < LimitDistance * LimitDistance)
			{
				Positions[ParticleIdx] += (RopeCenter - CollisionCenter).GetSafeNormal() * (LimitDistance - (RopeCenter - CollisionCenter).Size()) * CollisionProjectionAlpha;
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
			//float DistSquared = FMath::PointDistToSegmentSquared(RopeCenter, StartPoint, EndPoint);
			const FVector& ClosestPoint = FMath::ClosestPointOnSegment(RopeCenter, StartPoint, EndPoint);
			float DistSquared = (RopeCenter - ClosestPoint).SizeSquared();

			float LimitDistance = RopeRadius + CapsuleComp->GetScaledCapsuleRadius();
			if (DistSquared < LimitDistance * LimitDistance)
			{
				Positions[ParticleIdx] += (RopeCenter - ClosestPoint).GetSafeNormal() * (LimitDistance - (RopeCenter - ClosestPoint).Size()) * CollisionProjectionAlpha;
			}
			continue;
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

	for (TPair<TWeakObjectPtr<class UPrimitiveComponent>, FTransform> CollisionPose : RopeBlockerCollisionsPoseMap)
	{
		if (CollisionPose.Key == nullptr)
		{
			continue;
		}

		const FTransform& Pose = CollisionPose.Value;
		// 前フレームで同じコンポーネントがなければ、前フレームのポーズは現フレームのポーズとしておく
		const FTransform* PrevPosePtr = PrevRopeBlockerCollisionsPoseMap.Find(CollisionPose.Key);
		const FTransform& PrevPose = PrevPosePtr == nullptr ? Pose : *PrevPosePtr;
		// イテレーションごとの位置補間は線形補間で行う
		const FVector& CollisionCenter = FMath::Lerp(PrevPose.GetLocation(), Pose.GetLocation(), SubStepAlpha);

		// USphereComponentとUCapsuleComponent以外は今のところ非対応
		const USphereComponent* SphereComp = Cast<const USphereComponent>(CollisionPose.Key.Get());
		if (SphereComp != nullptr)
		{
			float LimitDistance = RopeRadius + SphereComp->GetScaledSphereRadius();
			if ((RopeCenter - CollisionCenter).SizeSquared() < LimitDistance * LimitDistance)
			{
				const FVector& CollisionVelocity = (Pose.GetLocation() - PrevPose.GetLocation()) / DeltaSeconds;
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
			//float DistSquared = FMath::PointDistToSegmentSquared(RopeCenter, StartPoint, EndPoint);
			const FVector& ClosestPoint = FMath::ClosestPointOnSegment(RopeCenter, StartPoint, EndPoint);
			float DistSquared = (RopeCenter - ClosestPoint).SizeSquared();

			float LimitDistance = RopeRadius + CapsuleComp->GetScaledCapsuleRadius();
			if (DistSquared < LimitDistance * LimitDistance)
			{
				// インパクトポイントの速度を求める
				const FVector& ClosestLocalPosition = CapsuleTM.InverseTransformPosition(ClosestPoint);
				const FVector& CenterToClosestPos = CapsuleTM.GetRotation().RotateVector(ClosestLocalPosition);
				const FQuat& RotDiff = Pose.GetRotation() * PrevPose.GetRotation().Inverse();
				const FVector& PrevCenterToClosestPos = RotDiff.UnrotateVector(CenterToClosestPos);
				const FVector& CollisionImpactPointVelocity = ((Pose.GetLocation() + CenterToClosestPos) - (PrevPose.GetLocation() + PrevCenterToClosestPos)) / DeltaSeconds;

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
			continue;
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

