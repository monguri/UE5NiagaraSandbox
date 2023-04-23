#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/AggregateGeom.h"
#include "RopeSimulatorCPU.generated.h"

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

private:
	UPROPERTY(EditAnywhere)
	float MeshScale = 1.0f;

	UPROPERTY(EditAnywhere)
	float RopeRadius = 1.0f;

	UPROPERTY(EditAnywhere)
	float RestLength = 2.7f;

	UPROPERTY(EditAnywhere)
	float Gravity = -981.0f;

	UPROPERTY(EditAnywhere)
	int32 NumThreads = 4;

	UPROPERTY(EditAnywhere)
	int32 NumParticles = 32;

	UPROPERTY(EditAnywhere)
	int32 NumSubStep = 1;

	UPROPERTY(EditAnywhere)
	int32 NumIteration = 4;

	UPROPERTY(EditAnywhere)
	float FrameRate = 60.0f;

	UPROPERTY(EditAnywhere)
	FBox WallBox = FBox(FVector(-150.0f, -150.0f, -150.0f), FVector(150.0f, 150.0f, 150.0f));

	UPROPERTY(EditAnywhere)
	float CollisionProjectionAlpha = 0.8f;

	UPROPERTY(EditAnywhere)
	float WallProjectionAlpha = 0.8f;

	UPROPERTY(EditAnywhere)
	float CollisionRestitution = 0.0f;

	UPROPERTY(EditAnywhere)
	float CollisionDynamicFriction = 0.0f;

	UPROPERTY(EditAnywhere)
	float WallRestitution = 0.0f;

	UPROPERTY(EditAnywhere)
	float WallDynamicFriction = 0.0f;

	UPROPERTY(EditAnywhere)
	TArray<TEnumAsByte<EObjectTypeQuery>> OverlapQueryObjectTypes;

	UPROPERTY(EditAnywhere)
	bool bConstraintEndPosition = false;

private:
	struct FCollisionCandidate
	{
		int32 AnotherParticleIdx;
		//FVector AnotherParticleToParticle;

		FCollisionCandidate(int32 _AnotherParticleIdx) : AnotherParticleIdx(_AnotherParticleIdx) {}
	};

	void UpdateRopeBlockers();
	void Integrate(int32 ParticleIdx, float SubStepDeltaSeconds);
	void SolvePositionConstraint(int32 InFrameExeCount);
	void SolveVelocity(float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount);
	void ApplyDistanceConstraint(int32 ParticleIdx);
	void ApplyRopeBlockersCollisionConstraint(int32 ParticleIdx, int32 InFrameExeCount);
	void ApplyWallCollisionConstraint(int32 ParticleIdx);
	void ApplyRopeBlockersVelocityConstraint(int32 ParticleIdx, float DeltaSeconds, float SubStepDeltaSeconds, int32 SubStepCount);
	void ApplyWallVelocityConstraint(int32 ParticleIdx, float SubStepDeltaSeconds);

private:
	TArray<FVector> Positions;
	TArray<FVector> PrevPositions;
	TArray<FVector> PrevCurrentIterationPositions;
	TArray<FVector> Velocities;
	TArray<FVector> PrevConstraintSolveVelocities;
	TArray<FVector> PrevSolveVelocities;
	TArray<FQuat> InitialOrientations;
	TArray<FQuat> Orientations;
	TArray<FLinearColor> Colors;
	// 加速度と慣性モーメントは毎フレーム計算するのでフレーム間のひきつぎはないのだが、使用メモリやTArrayの生成負荷をおさえるために
	// 使いまわしている
	TArray<FVector> Accelerations;
	TArray<FMatrix> InertiaInvs;
	int32 NumThreadParticles = 0;
	FTransform InvActorTransform = FTransform::Identity;

	UPROPERTY(Transient)
	TMap<TWeakObjectPtr<class UPrimitiveComponent>, FTransform> PrevRopeBlockerCollisionsPoseMap;

	UPROPERTY(Transient)
	TMap<TWeakObjectPtr<class UPrimitiveComponent>, FTransform> RopeBlockerCollisionsPoseMap;

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

