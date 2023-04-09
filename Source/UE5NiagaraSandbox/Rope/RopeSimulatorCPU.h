#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "RopeSimulatorCPU.generated.h"

UENUM()
enum class ERopeInitialDistribution : int32
{
	PlaneUniform,
	PlaneRandom,
};

UCLASS()
// ANiagaraActorを参考にしている
class ARopeSimulatorCPU : public AActor
{
	GENERATED_BODY()

public:
	ARopeSimulatorCPU();

	virtual void PostRegisterAllComponents() override;

	virtual void PreInitializeComponents() override;
	virtual void Tick( float DeltaSeconds ) override;

	/** Set true for this actor to self-destruct when the Niagara system finishes, false otherwise */
	UFUNCTION(BlueprintCallable)
	void SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish);

private:
	/** Pointer to System component */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, meta = (AllowPrivateAccess = "true"))
	class UNiagaraComponent* NiagaraComponent;

#if WITH_EDITORONLY_DATA
	// Reference to sprite visualization component
	UPROPERTY()
	class UBillboardComponent* SpriteComponent;

	// Reference to arrow visualization component
	UPROPERTY()
	class UArrowComponent* ArrowComponent;

#endif

private:
	/** True for this actor to self-destruct when the Niagara system finishes, false otherwise */
	UPROPERTY()
	uint32 bDestroyOnSystemFinish : 1;

	/** Callback when Niagara system finishes. */
	UFUNCTION(CallInEditor)
	void OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent);

	UPROPERTY(EditAnywhere)
	ERopeInitialDistribution InitialDistribution = ERopeInitialDistribution::PlaneUniform;

	UPROPERTY(EditAnywhere)
	float MeshScale = 6.0f;

	UPROPERTY(EditAnywhere)
	float Opacity = 1.0f;

	UPROPERTY(EditAnywhere)
	float RopeRadius = 1.0f;

	UPROPERTY(EditAnywhere)
	float Gravity = -981.0f;

	UPROPERTY(EditAnywhere)
	float InertiaMomentScale = 1.0f;

	UPROPERTY(EditAnywhere)
	int32 NumThreads = 4;

	UPROPERTY(EditAnywhere)
	int32 NumRopes = 100;

	UPROPERTY(EditAnywhere)
	int32 NumSubStep = 1;

	UPROPERTY(EditAnywhere)
	int32 NumIteration = 4;

	UPROPERTY(EditAnywhere)
	float FrameRate = 60.0f;

	UPROPERTY(EditAnywhere)
	FBox WallBox = FBox(FVector(-45.0f, -45.0f, -45.0f), FVector(45.0f, 45.0f, 45.0f));

	UPROPERTY(EditAnywhere)
	float CollisionProjectionAlpha = 0.8f;

	UPROPERTY(EditAnywhere)
	float CollisionRotProjectionAlpha = 0.5f;

	UPROPERTY(EditAnywhere)
	float WallProjectionAlpha = 0.8f;

	UPROPERTY(EditAnywhere)
	float WallRotProjectionAlpha = 0.5f;

	UPROPERTY(EditAnywhere)
	float CollisionRestitution = 0.0f;

	UPROPERTY(EditAnywhere)
	float CollisionDynamicFriction = 0.0f;

	UPROPERTY(EditAnywhere)
	float WallRestitution = 0.0f;

	UPROPERTY(EditAnywhere)
	float WallDynamicFriction = 0.0f;

	UPROPERTY(EditAnywhere)
	TArray<AActor*> CollisionActors;

	UPROPERTY(EditAnywhere)
	bool bAutoSpring = false;

	UPROPERTY(EditAnywhere)
	int32 SpringFrameInterval = 10;

	UPROPERTY(EditAnywhere, Meta=(UIMin = 1, UIMax = 4, ClampMin = 1, ClampMax = 4))
	int32 NumSpringPlace = 1;

	UFUNCTION(BlueprintCallable)
	void SpringOneRope(int32 SpringPlaceIdx = 0);

	UPROPERTY(EditAnywhere)
	float SleepCountTime = 2.0f;

	UPROPERTY(EditAnywhere)
	float SleepVelocity = 1.0f;

	UPROPERTY(EditAnywhere)
	float AwakeDistance = 100.0f;

	UPROPERTY(EditAnywhere)
	TArray<TEnumAsByte<EObjectTypeQuery>> OverlapQueryObjectTypes;

	UPROPERTY(EditAnywhere)
	bool bHideOnSleeping = false;

	UPROPERTY(EditAnywhere)
	int32 FadeFrameInterval = 30;

private:
	struct FCollisionCandidate
	{
		int32 AnotherParticleIdx;
		//FVector AnotherParticleToParticle;

		FCollisionCandidate(int32 _AnotherParticleIdx) : AnotherParticleIdx(_AnotherParticleIdx) {}
	};

	void UpdateActorCollisions();
	void UpdateCoinBlockers();
	bool IsCollisioned(const FVector& Position) const;
	void UpdateSleepState(float DeltaSeconds);
	void Integrate(int32 ParticleIdx, float SubStepDeltaSeconds);
	void SolvePositionConstraint(int32 InFrameExeCount);
	void SolveVelocity(float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount);
	void ApplyCollisionActorsConstraint(int32 ParticleIdx, int32 InFrameExeCount);
	void ApplyCoinBlockersCollisionConstraint(int32 ParticleIdx, int32 InFrameExeCount);
	void CalculateOneWallCollisionProjection(int32 ParticleIdx, const FPlane& WallPlane, FVector& InOutDeltaPos, FQuat& InOutDeltaRot);
	void ApplyWallCollisionConstraint(int32 ParticleIdx);
	void ApplyCollisionActorsVelocityConstraint(int32 ParticleIdx, float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount);
	void ApplyCoinBlockersVelocityConstraint(int32 ParticleIdx, float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount);
	void CalculateOneWallVelocityConstraint(int32 ParticleIdx, const FPlane& WallPlane, float SubStepDeltaSeconds, FVector& InOutDeltaPos, FVector& InOutDeltaRot);
	void ApplyWallVelocityConstraint(int32 ParticleIdx, float SubStepDeltaSeconds);

private:
	TArray<FVector> InitialPositions;
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<FVector> PrevConstraintSolvePositions;
	TArray<FQuat> Orientations;
	TArray<FQuat> PrevOrientations;
	TArray<FVector> Velocities;
	TArray<FVector> AngularVelocities;
	TArray<FVector> PrevConstraintSolveVelocities;
	TArray<FVector> PrevSolveVelocities;
	TArray<FVector> PrevConstraintSolveAngularVelocities;
	TArray<FVector> PrevSolveAngularVelocities;
	TArray<FLinearColor> Colors;
	// 加速度と慣性モーメントは毎フレーム計算するのでフレーム間のひきつぎはないのだが、使用メモリやTArrayの生成負荷をおさえるために
	// 使いまわしている
	TArray<FVector> Accelerations;
	TArray<FMatrix> InertiaInvs;
	TArray<bool> bSleeps;
	TArray<float> NotMovingTimes;
	TArray<int32> FadeFrameCounters;
	int32 NumThreadParticles = 0;
	FTransform InvActorTransform = FTransform::Identity;
	TArray<FKAggregateGeom> PrevActorsAggGeom;
	TArray<FKAggregateGeom> ActorsAggGeom;
	int32 SpringRopeIndex = 0;
	int32 SpringFrameCounter = 0;

	UPROPERTY(Transient)
	TMap<TWeakObjectPtr<class UPrimitiveComponent>, FTransform> PrevCoinBlockerCollisionsPoseMap;

	UPROPERTY(Transient)
	TMap<TWeakObjectPtr<class UPrimitiveComponent>, FTransform> CoinBlockerCollisionsPoseMap;

public:
	/** Returns NiagaraComponent subobject **/
	class UNiagaraComponent* GetNiagaraComponent() const { return NiagaraComponent; }
#if WITH_EDITORONLY_DATA
	/** Returns SpriteComponent subobject **/
	class UBillboardComponent* GetSpriteComponent() const { return SpriteComponent; }
	/** Returns ArrowComponent subobject **/
	class UArrowComponent* GetArrowComponent() const { return ArrowComponent; }
#endif

#if WITH_EDITOR
	// AActor interface
	virtual bool GetReferencedContentObjects(TArray<UObject*>& Objects) const override;
	// End of AActor interface

	/** Reset this actor in the level.*/
	void ResetInLevel();
#endif // WITH_EDITOR
};

