#include "TautRopeSimulatorCPU.h"
#include "UObject/ConstructorHelpers.h"
#include "Engine/Texture2D.h"
#include "Components/ArrowComponent.h"
#include "Components/BillboardComponent.h"
#include "NiagaraComponent.h"
#include "NiagaraSystem.h"
#include "NiagaraFunctionLibrary.h"
#include "NiagaraDataInterfaceArrayFunctionLibrary.h"
#include "Async/ParallelFor.h"
#include "Chaos/Convex.h"
#include "PhysicsInterfaceTypesCore.h"

void ATautRopeSimulatorCPU::PreInitializeComponents()
{
	Super::PreInitializeComponents();

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();

	// まずは一つの線分から
	NumParticles = 2;
	NumSegments = NumParticles - 1;

	Positions.SetNum(NumParticles);
	PrevPositions.SetNum(NumParticles);
	ParentPositions.SetNum(NumSegments);
	ChildPositions.SetNum(NumSegments);
	Colors.SetNum(NumParticles - 1);

	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ParticleIdx++)
	{
		PrevPositions[ParticleIdx] = PrevPositions[ParticleIdx] = Positions[ParticleIdx] = FVector::XAxisVector * 100.0f * ParticleIdx; // 1mの長さの線分
#if 0
		if (ParticleIdx % 2 == 0)
		{
			Orientations[ParticleIdx] = InitialOrientations[ParticleIdx] = FQuat(FVector::ZAxisVector, PI / 2) * FQuat(FVector::XAxisVector, PI / 2);
		}
		else
		{
			Orientations[ParticleIdx] = InitialOrientations[ParticleIdx] = FQuat(FVector::XAxisVector, PI / 2);
		}
#endif
	}

	for (int32 SegmentIdx = 0; SegmentIdx < NumSegments; SegmentIdx++)
	{
		Colors[SegmentIdx] = FLinearColor::MakeRandomColor();
		ParentPositions[SegmentIdx] = Positions[SegmentIdx];
		ChildPositions[SegmentIdx] = Positions[SegmentIdx + 1];
	}

	// Tick()で設定しても、レベルにNiagaraSystemが最初から配置されていると、初回のスポーンでは配列は初期値を使ってしまい
	//間に合わないのでBeginPlay()でも設定する

	// パーティクル数でなくセグメント数をNumParticlesには設定する
	NiagaraComponent->SetNiagaraVariableInt("NumSegments", NumSegments);

	NiagaraComponent->SetNiagaraVariableFloat("RopeRadius", RopeRadius);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ParentPositions"), ParentPositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ChildPositions"), ChildPositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayColor(NiagaraComponent, FName("Colors"), Colors);
}

void ATautRopeSimulatorCPU::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ParticleIdx++)
		{
			PrevPositions[ParticleIdx] = Positions[ParticleIdx];
		}

		if (StartConstraintActor != nullptr)
		{
			// StartConstraintActorが設定されていればルートをコンストレイント
			// とりあえずGetActorUpVector()にStartConstraintRadiusだけ離れた位置にコンストレイントさせる形にする
			Positions[0] = InvActorTransform.TransformPosition(StartConstraintActor->GetActorLocation() + StartConstraintActor->GetActorUpVector() * StartConstraintRadius);
		}
		if (EndConstraintActor != nullptr)
		{
			// EndConstraintActorが設定されていれば末端をコンストレイント
			// とりあえずGetActorUpVector()にEndConstraintRadiusだけ離れた位置にコンストレイントさせる形にする
			Positions[Positions.Num() - 1] = InvActorTransform.TransformPosition(EndConstraintActor->GetActorLocation() + EndConstraintActor->GetActorUpVector() * EndConstraintRadius);
		}
		//TODO:長さ制限を与えてConstraintActor自体をコンストレイントさせるのは後で行う:w

		UpdateRopeBlockers();
		SolveRopeBlockersCollisionConstraint();
	}

	for (int32 SegmentIdx = 0; SegmentIdx < NumSegments; SegmentIdx++)
	{
		ParentPositions[SegmentIdx] = Positions[SegmentIdx];
		ChildPositions[SegmentIdx] = Positions[SegmentIdx + 1];
	}
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ParentPositions"), ParentPositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ChildPositions"), ChildPositions);

#if 0
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
#endif
}

void ATautRopeSimulatorCPU::UpdateRopeBlockers()
{
	TArray<FOverlapResult> Overlaps;
	FCollisionObjectQueryParams ObjectParams(OverlapQueryObjectTypes);
	if (!ObjectParams.IsValid())
	{
		return;
	}

	FCollisionQueryParams Params(SCENE_QUERY_STAT(UpdateRopeBlockers), false);

	GetWorld()->OverlapMultiByObjectType(Overlaps, GetActorLocation() + OverlapQueryBox.GetCenter(), GetActorQuat(), ObjectParams, FCollisionShape::MakeBox(OverlapQueryBox.GetExtent()), Params);

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
		RopeBlockerAggGeomMap.Add(Primitive, OriginalAggGeom);

		// FKAggregateGeomのコピーコンストラクトを少なくしたいのでマップにAddしてから編集する
		// TODO:コピーは結構重いだろう。FKConvexElemがあるので
		FKAggregateGeom* CacheAggGeom = RopeBlockerAggGeomMap.Find(Primitive);

		// キャッシュするFKAggregateGeomにはActorのTransformを先に適用しておく。
		// PrevRopeBlockerAggGeomMapの状態と各コンストレイントで比較する必要があるため。
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

		// FKConvexElemはTransformを保持する形なので直接保存しておく
		for (int32 i = 0; i < OriginalAggGeom.ConvexElems.Num(); ++i)
		{
			CacheAggGeom->ConvexElems[i].SetTransform(ActorTM);
			// FKConvexElem::SetChaosConvexMesh()は所有権の移動になるので、一から作り直すしかない
			// UOceanCollisionComponent::UpdateBodySetup()、UOceanCollisionComponent::UpdateBodySetup()が似たようなことをやっている
			int32 NumHullVerts = CacheAggGeom->ConvexElems[i].VertexData.Num();
			TArray<Chaos::FConvex::FVec3Type> ConvexVertices;
			ConvexVertices.SetNum(NumHullVerts);
			// TODO:これ、TArray同士のコピーなんで=でダメ？
			for (int32 VertIndex = 0; VertIndex < NumHullVerts; ++VertIndex)
			{
				ConvexVertices[VertIndex] = CacheAggGeom->ConvexElems[i].VertexData[VertIndex];
			}

			TSharedPtr<Chaos::FConvex, ESPMode::ThreadSafe> ChaosConvex = MakeShared<Chaos::FConvex, ESPMode::ThreadSafe>(ConvexVertices, 0.0f);

			//TODO:重い。Chaos::FConvexBuilder::BuildIndices()を走らせている
			CacheAggGeom->ConvexElems[i].SetChaosConvexMesh(MoveTemp(ChaosConvex));
		}
	}
}

void ATautRopeSimulatorCPU::SolveRopeBlockersCollisionConstraint()
{
}

ATautRopeSimulatorCPU::ATautRopeSimulatorCPU()
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

void ATautRopeSimulatorCPU::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	// Set Notification Delegate
	if (NiagaraComponent)
	{
		NiagaraComponent->OnSystemFinished.AddUniqueDynamic(this, &ATautRopeSimulatorCPU::OnNiagaraSystemFinished);
	}
}

void ATautRopeSimulatorCPU::SetDestroyOnSystemFinish(bool bShouldDestroyOnSystemFinish)
{
	bDestroyOnSystemFinish = bShouldDestroyOnSystemFinish ? 1 : 0;  
};

void ATautRopeSimulatorCPU::OnNiagaraSystemFinished(UNiagaraComponent* FinishedComponent)
{
	if (bDestroyOnSystemFinish)
	{
		SetLifeSpan(0.0001f);
	}
}

#if WITH_EDITOR
bool ATautRopeSimulatorCPU::GetReferencedContentObjects(TArray<UObject*>& Objects) const
{
	Super::GetReferencedContentObjects(Objects);

	if (UNiagaraSystem* System = NiagaraComponent->GetAsset())
	{
		Objects.Add(System);
	}

	return true;
}

void ATautRopeSimulatorCPU::ResetInLevel()
{
	if (NiagaraComponent)
	{
		NiagaraComponent->Activate(true);
		NiagaraComponent->ReregisterComponent();
	}
}
#endif // WITH_EDITOR

