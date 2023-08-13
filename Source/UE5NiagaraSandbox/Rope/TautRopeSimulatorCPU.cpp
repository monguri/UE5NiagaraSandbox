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
#include "Chaos/TriangleMeshImplicitObject.h"
#include "DrawDebugHelpers.h"

void ATautRopeSimulatorCPU::BeginPlay()
{
	Super::BeginPlay();

	// 何度も使うのでキャッシュしておく
	ToleranceSquared = Tolerance * Tolerance; // TODO:リアルタイムには更新しない

	// まずは一つの線分から
	int32 NumParticles = 2;
	int32 NumSegments = NumParticles - 1;

	Positions.SetNum(NumParticles);
	PrevPositions.SetNum(NumParticles);
	ParentPositions.SetNum(NumSegments);
	ChildPositions.SetNum(NumSegments);
	EdgeIdxOfPositions.SetNum(NumParticles);
	MovedFlagOfPositions.SetNum(NumParticles);

	UpdateStartEndConstraint();

	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles; ParticleIdx++)
	{
		PrevPositions[ParticleIdx] = Positions[ParticleIdx];
		EdgeIdxOfPositions[ParticleIdx] = INDEX_NONE;
		MovedFlagOfPositions[ParticleIdx] = false;
	}

	for (int32 ParticleIdx = 0; ParticleIdx < NumParticles - 1; ParticleIdx++)
	{
		ParentPositions[ParticleIdx] = Positions[ParticleIdx];
		ChildPositions[ParticleIdx] = Positions[ParticleIdx + 1];
	}
}

void ATautRopeSimulatorCPU::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);

	// 何度も使うのでキャッシュしておく
	InvActorTransform = GetActorTransform().Inverse();

	if (DeltaSeconds > KINDA_SMALL_NUMBER)
	{
		UpdateStartEndConstraint();
		UpdateRopeBlockers();
		//SolveRopeBlockersCollisionConstraintOld();
		SolveRopeBlockersCollisionConstraint();
	}

	check(Positions.Num() >= 2);
	check(Positions.Num() == PrevPositions.Num());
	// 直接サイズとインデックス指定でコピーする方法がないので一旦別変数にコピーしてMove。
	int32 NumSegments = Positions.Num() - 1;
	TArray<FVector> TmpParentPositions(Positions.GetData(), NumSegments);
	TArray<FVector> TmpChildPositions(&Positions.GetData()[1], NumSegments);
	ParentPositions = MoveTemp(TmpParentPositions);
	ChildPositions = MoveTemp(TmpChildPositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ParentPositions"), ParentPositions);
	UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("ChildPositions"), ChildPositions);

	if (bDrawCollisionEdge)
	{
		// TODO:これもいっそNiagaraで描画するか？
		for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
		{
			const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
			const FVector& LineStartWS = GetActorTransform().TransformPosition(Edge.Key);
			const FVector& LineEndWS = GetActorTransform().TransformPosition(Edge.Value);
			DrawDebugLine(GetWorld(), LineStartWS, LineEndWS, FLinearColor::Red.ToFColorSRGB());
		}

		EdgeIdxDebugDrawPositions.Reset();
		for (const TPair<FVector, FVector>& Pair : RopeBlockerTriMeshEdgeArray)
		{
			EdgeIdxDebugDrawPositions.Add((Pair.Key + Pair.Value) * 0.5);
		}
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("EdgeIdxDebugDrawPositions"), EdgeIdxDebugDrawPositions);
	}
	else
	{
		// 空配列を渡すことでエッジのデバッグ描画用のパーティクルを0にする
		UNiagaraDataInterfaceArrayFunctionLibrary::SetNiagaraArrayVector(NiagaraComponent, FName("EdgeIdxDebugDrawPositions"), TArray<FVector>());
	}

	if (bDrawTensionDir)
	{
		for (int32 ParticleIdx = 1; ParticleIdx < Positions.Num() - 1; ParticleIdx++)
		{
			// TODO:頂点間がToleranceより近い場合は頂点をまとめるべきなのでこのTolerance処理は必要ない
			const FVector& PreSegmentDir = (Positions[ParticleIdx] - Positions[ParticleIdx - 1]).GetSafeNormal(Tolerance);
			const FVector& PostSegmentDir = (Positions[ParticleIdx + 1] - Positions[ParticleIdx]).GetSafeNormal(Tolerance);
			const FVector& TensionDir = (-PreSegmentDir + PostSegmentDir).GetSafeNormal(Tolerance);

			const FVector& LineStartWS = GetActorTransform().TransformPosition(Positions[ParticleIdx]);
			const FVector& LineEndWS = GetActorTransform().TransformPosition(Positions[ParticleIdx] + TensionDir * 100); // TODO:適当に1メートル
			DrawDebugLine(GetWorld(), LineStartWS, LineEndWS, FLinearColor::Blue.ToFColorSRGB(), false, -1.f, ESceneDepthPriorityGroup::SDPG_Foreground);
		}
	}
}

void ATautRopeSimulatorCPU::UpdateStartEndConstraint()
{
	if (StartConstraintComponent != nullptr)
	{
		Positions[0] = InvActorTransform.TransformPosition(StartConstraintComponent->GetSocketLocation(StartConstraintSocket));
	}
	else if (StartConstraintActor != nullptr)
	{
		Positions[0] = InvActorTransform.TransformPosition(StartConstraintActor->GetRootComponent()->GetSocketLocation(StartConstraintSocket));
	}

	if (EndConstraintComponent != nullptr)
	{
		Positions[Positions.Num() - 1] = InvActorTransform.TransformPosition(EndConstraintComponent->GetSocketLocation(EndConstraintSocket));
	}
	else if (EndConstraintActor != nullptr)
	{
		Positions[Positions.Num() - 1] = InvActorTransform.TransformPosition(EndConstraintActor->GetRootComponent()->GetSocketLocation(EndConstraintSocket));
	}
	//TODO:長さ制限を与えてConstraintActor自体をコンストレイントさせるのは後で行う
}

void ATautRopeSimulatorCPU::UpdateRopeBlockers()
{
	TArray<FOverlapResult> Overlaps;
	FCollisionObjectQueryParams ObjectParams(OverlapQueryObjectTypes);
	if (!ObjectParams.IsValid())
	{
		return;
	}

	// OverlapクエリはSimpleコリジョンへのクエリで十分と判断
	FCollisionQueryParams Params(SCENE_QUERY_STAT(UpdateRopeBlockers), false);

	GetWorld()->OverlapMultiByObjectType(Overlaps, GetActorLocation() + OverlapQueryBox.GetCenter(), GetActorQuat(), ObjectParams, FCollisionShape::MakeBox(OverlapQueryBox.GetExtent()), Params);

	// コリジョンがPrimitiveComponent単位で増減することを想定し、呼ばれるたびにマップを作り直す
	//TODO: PrimitiveComponent内のコリジョンシェイプの増減は想定してない
	OverlapPrimitives.Reset();
	RopeBlockerTriMeshEdgeArray.Reset();

	for (const FOverlapResult& Overlap : Overlaps)
	{
		UPrimitiveComponent* Primitive = Overlap.GetComponent();
		if (Primitive == nullptr || Primitive->GetBodyInstance() == nullptr || Primitive->GetBodyInstance()->GetBodySetup() == nullptr)
		{
			continue;
		}

		OverlapPrimitives.Add(Primitive);

		// ローカル座標に変換
		const FTransform& ActorTM = Primitive->GetComponentTransform() * InvActorTransform;

		//TODO:毎フレームエッジ配列を作る必要はまずない
		TArray<TPair<FVector, FVector>> EdgeArray;

		// TODO:ChaosTriMeshesにも複数のFTriangleMeshImplicitObjectがあるのにすべてまとめてEdgeArrayに入れるのは
		// 無理。
		for (const TSharedPtr<Chaos::FTriangleMeshImplicitObject, ESPMode::ThreadSafe>& TriMesh : Primitive->GetBodyInstance()->GetBodySetup()->ChaosTriMeshes)
		{
			//TODO: 名前空間関数で作りたいが、LargeIdxTypeとSmallIdxTypeの両方で共通の関数を作るのが難しい
			auto CollectEdges = [&](const auto& Triangles)
			{
				// 検索を高速にするためハッシュ構造のあるTSetを使う
				TSet<TPair<int32, int32>> UniqueEdgeSet;

				// 上限サイズをとっておく
				int32 NumTris = Triangles.Num();
				UniqueEdgeSet.Reserve(NumTris * 3);
				EdgeArray.Reserve(NumTris * 3);

				const Chaos::FTriangleMeshImplicitObject::ParticlesType& Vertices = TriMesh->Particles();
		
				// ユニークなエッジ配列を作る処理はFTriangleMesh::GetSegmentMesh()を参考にした
				for (int32 TriIdx = 0; TriIdx < NumTris; TriIdx++)
				{
					for (int32 VertIdx = 0; VertIdx < 3; VertIdx++)
					{
						int32 EdgeIdx0 = Triangles[TriIdx][VertIdx];
						int32 EdgeIdx1 = Triangles[TriIdx][(VertIdx + 1) % 3];
						TPair<int32, int32> Edge;
						if (EdgeIdx0 <= EdgeIdx1)
						{
							Edge = TPair<int32, int32>(EdgeIdx0, EdgeIdx1);
						}
						else
						{
							Edge = TPair<int32, int32>(EdgeIdx1, EdgeIdx0);
						}

						const TPair<int32, int32>* FoundEdge = UniqueEdgeSet.Find(Edge);
						if (FoundEdge == nullptr)
						{
							UniqueEdgeSet.Add(Edge);

							EdgeArray.Add(
								TPair<FVector, FVector>(
									ActorTM.TransformPosition(FVector(Vertices.X(Edge.Key))),
									ActorTM.TransformPosition(FVector(Vertices.X(Edge.Value)))
								)
							);
						}
					}
				}
			};

			const Chaos::FTrimeshIndexBuffer& IdxBuffer = TriMesh->Elements();
			if (IdxBuffer.RequiresLargeIndices())
			{
				CollectEdges(IdxBuffer.GetLargeIndexBuffer());
			}
			else
			{
				CollectEdges(IdxBuffer.GetSmallIndexBuffer());
			}
		}

		//TODO:CollisionStatesで使うために、毎フレーム変化がないこと、要素の順番に変更がないことを前提にしている
		RopeBlockerTriMeshEdgeArray.Append(EdgeArray);
	}
}

namespace NiagaraSandbox::RopeSimulator
{
	//TODO: FMath::PointDistToSegmentSquared()が結果をfloatにキャストして戻り値にしてるのでしょうがなく
	double PointDistToSegmentSquared(const FVector &Point, const FVector &StartPoint, const FVector &EndPoint)
	{
		const FVector& ClosestPoint = FMath::ClosestPointOnSegment(Point, StartPoint, EndPoint);
		return (Point - ClosestPoint).SizeSquared();
	}
}

void ATautRopeSimulatorCPU::SolveRopeBlockersCollisionConstraintOld()
{
	using namespace NiagaraSandbox::RopeSimulator;

	// TODO:これもいっそNiagaraで描画するか？
	if (bDrawCollisionEdge)
	{
		for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
		{
			const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
			const FVector& LineStartWS = GetActorTransform().TransformPosition(Edge.Key);
			const FVector& LineEndWS = GetActorTransform().TransformPosition(Edge.Value);
			DrawDebugLine(GetWorld(), LineStartWS, LineEndWS, FLinearColor::Red.ToFColorSRGB());
		}
	}

	// MovementPhaseとCollisionPhaseの全頂点のイテレーション。収束するまでループする。
	bool bExistMovedParticle = false;
	if (Positions[0] != PrevPositions[0]
		|| Positions[Positions.Num() - 1] != PrevPositions[Positions.Num() - 1]) // TODO:Toleranceを入れないでみる
	{
		bExistMovedParticle = true;
	}

	//
	// イテレーションループ　ここから
	//
	for (int32 IterCount = 0; IterCount < MaxIteration && bExistMovedParticle; IterCount++)
	{
		bool bExistAddedParticle = false;

		//
		// パーティクルループ　ここから
		//
		// TODO: 逆方向ループはあとで必要か検討する
		for (int32 ParticleIdx = 0; ParticleIdx < Positions.Num(); ParticleIdx = ((bExistAddedParticle && ParticleIdx == 0) ? 0 : ParticleIdx + 1)) // 始点のときのみ頂点追加があったらもう一度始点で行う
		{
			bool bNeedCollisionPhase = false;
			MovedFlagOfPositions[ParticleIdx] = false;

			//
			// MovementPhase　ここから
			//
			{
				if (ParticleIdx == 0)
				{
					bNeedCollisionPhase = (Positions[0] != PrevPositions[0]); // TODO:Toleranceを入れないでみる

					// 始点と終点は1フレームに一度しか動かさないので即Movedフラグを下げる
					MovedFlagOfPositions[ParticleIdx] = false;
				}
				else if (ParticleIdx == (Positions.Num() - 1))
				{
					bNeedCollisionPhase = (Positions[Positions.Num() - 1] != PrevPositions[Positions.Num() - 1]); // TODO:Toleranceを入れないでみる

					// 始点と終点は1フレームに一度しか動かさないので即Movedフラグを下げる
					MovedFlagOfPositions[ParticleIdx] = false;
				}
				else // 始点と終点以外は最短コンストレイント
				{
					check(EdgeIdxOfPositions[ParticleIdx] != INDEX_NONE);
					const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdxOfPositions[ParticleIdx]];
					const FVector& EdgeStart = Edge.Key;
					const FVector& EdgeEnd = Edge.Value;

					const FVector& StartPoint = Positions[ParticleIdx - 1];
					const FVector& EndPoint = Positions[ParticleIdx + 1];

					// 動いてるのがStartPointであろうとEndPointであろうとStartPointPlane側を回転させて
					// 最短交点を計算する

					// エッジの直線に垂線を下ろした点をエッジを直線にして計算
					const FVector& StartPointDropFoot = FMath::ClosestPointOnInfiniteLine(EdgeStart, EdgeEnd, StartPoint);
					const FVector& EndPointDropFoot = FMath::ClosestPointOnInfiniteLine(EdgeStart, EdgeEnd, EndPoint);

					double StartPointPerpLen = (StartPoint - StartPointDropFoot).Size();
					double EndPointPerpLen = (EndPoint - EndPointDropFoot).Size();
					
					// 最短になる交点は垂線の長さの比による線形補間で決まる
					double Alpha = StartPointPerpLen / (StartPointPerpLen + EndPointPerpLen);
					const FVector& ShortestPointOnInfiniteLine = FMath::Lerp(StartPointDropFoot, EndPointDropFoot, Alpha);
					double EdgeLenSqr = FVector::DotProduct(EdgeEnd - EdgeStart, EdgeEnd - EdgeStart);

					// エッジは線分なのでクランプする
					bool bClamped = false;
					if (FVector::DotProduct(ShortestPointOnInfiniteLine - EdgeStart, EdgeEnd - EdgeStart) > EdgeLenSqr) // TODO:Toleranceを入れないでみる
					{
						Positions[ParticleIdx] = EdgeEnd;
						bClamped = true;
					}
					else if (FVector::DotProduct(ShortestPointOnInfiniteLine - EdgeEnd, EdgeStart - EdgeEnd) > EdgeLenSqr) // TODO:Toleranceを入れないでみる
					{
						Positions[ParticleIdx] = EdgeStart;
						bClamped = true;
					}
					else
					{
						Positions[ParticleIdx] = ShortestPointOnInfiniteLine;
						bClamped = false;
					}

					// エッジの端にクランプされたとき、動いていたらCollisionPhaseへ。属すべきエッジの変化が起きうるので、エッジ移動、頂点削除、追加などの判定を行う。
					bNeedCollisionPhase = bClamped && (Positions[ParticleIdx] != PrevPositions[ParticleIdx]); // TODO:Toleranceを入れないでみる

					// 収束のため閾値つきで動いたかどうか判定
					MovedFlagOfPositions[ParticleIdx] = ((PrevPositions[ParticleIdx] - Positions[ParticleIdx]).SizeSquared() > ToleranceSquared);
				}
			}
			//
			// MovementPhase　ここまで
			//

			// 追加頂点があると始点終点はその延長までしか動かさず再度MovementPhaseをやるので延長位置を保存する
			bExistAddedParticle = false;

			//
			// CollisionPhase　ここから
			//
			if (bNeedCollisionPhase)
			{
				//
				// 始点セグメントのCollisionPhase　ここから
				// 
				// 単に動いた時の前後セグメントでの頂点追加
				// TODO:エッジ移動とエッジに沿った削除はあとで実装する
				// TODO:現状、頂点追加は始点と終点のセグメント以外では考えない
				if (ParticleIdx == 0)
				{
					// 動いている頂点
					const FVector& TriVert0 = PrevPositions[ParticleIdx];
					const FVector& TriVert1 = Positions[ParticleIdx];
					const FVector& TriVert2 = PrevPositions[ParticleIdx + 1];

					double NearestEdgeDistanceSq = DBL_MAX;
					FVector NearestIntersectPoint = FVector::ZeroVector;
					int32 NearestEdgeIdx = INDEX_NONE;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						// 異なるエッジでないものは採用しない
						if (EdgeIdxOfPositions[ParticleIdx] == EdgeIdx || EdgeIdxOfPositions[ParticleIdx + 1] == EdgeIdx)
						{
							continue;
						}

						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& RayStart = Edge.Key;
						const FVector& RayEnd = Edge.Value;
						FVector IntersectPoint;
						FVector IntersectNormal;
						bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
						if (bIntersecting)
						{
							// 既存の動いてない方の頂点と近すぎるものは追加しない。複数エッジの交点と衝突した場合に複数頂点追加しうるので
							if ((IntersectPoint - TriVert2).SizeSquared() > ToleranceSquared)
							{
								// 元の線分と最も近いエッジを採用
								double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(IntersectPoint, TriVert0, TriVert2);
								if (EdgeDistanceSq < NearestEdgeDistanceSq)
								{
									// 等距離なら最も若いインデックスを採用する
									NearestEdgeDistanceSq = EdgeDistanceSq;
									NearestIntersectPoint = IntersectPoint;
									NearestEdgeIdx = EdgeIdx;
								}
							}
						}
					}

					// 頂点追加
					if (NearestEdgeIdx != INDEX_NONE)
					{
						// TODO:GDC動画のように延長線上を新たな終点におくと、同一平面に複数エッジがある
						// シェイプだと、同一平面上のエッジを拾い損ねることがFMath::SegmentTriangleIntersection()では簡単に起きる。
						// Triangleの辺にエッジがある場合に交差を検出失敗する。
						// よってPrevPositionsの更新は頂点の追加が終わるまでしない
						PrevPositions.Insert(NearestIntersectPoint, ParticleIdx + 1);
						Positions.Insert(NearestIntersectPoint, ParticleIdx + 1);
						EdgeIdxOfPositions.Insert(NearestEdgeIdx, ParticleIdx + 1);
						MovedFlagOfPositions.Insert(true, ParticleIdx + 1); // ここでtrueにしてもパーティクルループの自分の番のMovementPhaseで別途判定される

						// MoveHalfwayPositionからPositions[ParticleIdx]の間で動かしてさらに他のエッジ接触がないかチェックする
						bExistAddedParticle = true;
					}

					// エッジからはがす削除
					// TODO:終点と処理が冗長
					if (Positions.Num() >= 3)
					{
						// 0-2の線分の削除だけで判定していないのは、なにかに1をひっかけている状態で
						// 0-2を大きく動かすと0-2線分上にコリジョンがいない状態になり削除対象になってしまうから
						// TODO:本当は3つのライントレースでなくTriangleとコリジョンのOverlapをとりたかったがAPIがなかった
						// TODO:もっといいやり方あるかも

						bool bTraceComplex = false;
						FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(SolveRopeBlockersCollisionConstraint), bTraceComplex);
						// Toleranceだけ0と2を双方から縮めるのは、どちらかあるいは両方がエッジと接触
						// していたらそのエッジとヒット判定になり、1の削除判定をしたいのにできないため
						// TODO:もっといい方法ある？
						// Toleranceだけ1を0-2の線分側に近づけるのは、1がエッジと接触してると接触判定し続けるため
						// TODO:もっといい方法ある？
						const FVector& SmallTriVert0 = Positions[2] + (Positions[0] - Positions[2]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert1 = Positions[1] + ((Positions[2] + Positions[0]) * 0.5 - Positions[1]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert2 = Positions[0] + (Positions[2] - Positions[0]).GetSafeNormal() * Tolerance;

						const FVector& SmallTriVertWS0 = GetActorTransform().TransformPosition(SmallTriVert0);
						const FVector& SmallTriVertWS1 = GetActorTransform().TransformPosition(SmallTriVert1);
						const FVector& SmallTriVertWS2 = GetActorTransform().TransformPosition(SmallTriVert2);

						bool bIntersectionExist = false;
						for (UPrimitiveComponent* Primitive : OverlapPrimitives)
						{
							FHitResult HitResult;
							bool bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS0, SmallTriVertWS1, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS0, bHit ? HitResult.Location : SmallTriVertWS1, FColor(255, 128, 0), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS1, FColor(0, 128, 255), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS1, SmallTriVertWS2, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS1, bHit ? HitResult.Location : SmallTriVertWS2, FColor(255, 128, 0), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS2, FColor(0, 128, 255), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS2, SmallTriVertWS0, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS2, bHit ? HitResult.Location : SmallTriVertWS0, FColor(255, 128, 0), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS0, FColor(0, 128, 255), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}
						}

						if (!bIntersectionExist)
						{
							PrevPositions.RemoveAt(1);
							Positions.RemoveAt(1);
							EdgeIdxOfPositions.RemoveAt(1);
							MovedFlagOfPositions.RemoveAt(1);

							// 追加したものが即削除された場合ももう追加判定はとめて次のパーティクルに進む。
							// 追加と即削除を繰り返すとParticleIdx=0のままでループが進まないので。
							bExistAddedParticle = false;
						}
					}

					if (!bExistAddedParticle)
					{
						PrevPositions[0] = Positions[0];
					}
				}
				//
				// 始点セグメントのCollisionPhase　ここまで
				// 

				//
				// 終点セグメントのCollisionPhase　ここから
				// 
				else if (ParticleIdx == (Positions.Num() - 1))
				{
					// 動いている頂点
					const FVector& TriVert1 = Positions[ParticleIdx];
					const FVector& TriVert2 = PrevPositions[ParticleIdx];
					
					// 固定している頂点は始点の判定と違ってPrevPositonsでなくPositionsなのに注意。
					// こうしないと1フレームでParticlesIdxの頂点が大きく動いたとき交差検出が漏れるケースがある
					// https://www.gdcvault.com/play/1027351/Rope-Simulation-in-Uncharted-4
					// の32分ごろの例。
					const FVector& TriVert0 = Positions[ParticleIdx - 1];

					// TODO: 上のifブロックと処理が冗長
					double NearestEdgeDistanceSq = DBL_MAX;
					FVector NearestIntersectPoint = FVector::ZeroVector;
					int32 NearestEdgeIdx = INDEX_NONE;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						// 異なるエッジでないものは採用しない
						if (EdgeIdxOfPositions[ParticleIdx - 1] == EdgeIdx || EdgeIdxOfPositions[ParticleIdx] == EdgeIdx)
						{
							continue;
						}

						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& RayStart = Edge.Key;
						const FVector& RayEnd = Edge.Value;
						FVector IntersectPoint;
						FVector IntersectNormal;
						bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
						if (bIntersecting)
						{
							// 既存の動いてない方の頂点と近すぎるものは追加しない。複数エッジの交点と衝突した場合に複数頂点追加しうるので
							if ((IntersectPoint - TriVert0).SizeSquared() > ToleranceSquared)
							{
								// 元の線分と最も近いエッジを採用
								double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(IntersectPoint, TriVert0, TriVert2);
								if (EdgeDistanceSq < NearestEdgeDistanceSq)
								{
									// 等距離なら最も若いインデックスを採用する
									NearestEdgeDistanceSq = EdgeDistanceSq;
									NearestIntersectPoint = IntersectPoint;
									NearestEdgeIdx = EdgeIdx;
								}
							}
						}
					}

					if (NearestEdgeIdx != INDEX_NONE)
					{
						// TODO:GDC動画のように延長線上を新たな終点におくと、同一平面に複数エッジがある
						// シェイプだと、同一平面上のエッジを拾い損ねることがFMath::SegmentTriangleIntersection()では簡単に起きる。
						// Triangleの辺にエッジがある場合に交差を検出失敗する。
						// よってPrevPositionsの更新は頂点の追加が終わるまでしない
						PrevPositions.Insert(NearestIntersectPoint, ParticleIdx);
						Positions.Insert(NearestIntersectPoint, ParticleIdx);
						EdgeIdxOfPositions.Insert(NearestEdgeIdx, ParticleIdx);
						MovedFlagOfPositions.Insert(true, ParticleIdx); // ここでtrueにして次イテレーションで最短コンストレイントを行う // TODO:逆順ループを作れば次イテレーションにしなくてもよくなり収束も早くなるかも

						// MoveHalfwayPositionからPositions[ParticleIdx]の間で動かしてさらに他のエッジ接触がないかチェックする
						bExistAddedParticle = true;
					}

					// エッジからはがす削除
					// TODO:終点と処理が冗長
					if (Positions.Num() >= 3)
					{
						// 0-2の線分の削除だけで判定していないのは、なにかに1をひっかけている状態で
						// 0-2を大きく動かすと0-2線分上にコリジョンがいない状態になり削除対象になってしまうから
						// TODO:本当は3つのライントレースでなくTriangleとコリジョンのOverlapをとりたかったがAPIがなかった
						// TODO:もっといいやり方あるかも

						bool bTraceComplex = false;
						FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(SolveRopeBlockersCollisionConstraint), bTraceComplex);
						// Toleranceだけ0と2を双方から縮めるのは、どちらかあるいは両方がエッジと接触
						// していたらそのエッジとヒット判定になり、1の削除判定をしたいのにできないため
						// TODO:もっといい方法ある？
						// Toleranceだけ1を0-2の線分側に近づけるのは、1がエッジと接触してると接触判定し続けるため
						// TODO:もっといい方法ある？
						const FVector& SmallTriVert0 = Positions[Positions.Num() - 1] + (Positions[Positions.Num() - 3] - Positions[Positions.Num() - 1]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert1 = Positions[Positions.Num() - 1 - 1] + ((Positions[Positions.Num() - 1] + Positions[Positions.Num() - 3]) * 0.5 - Positions[Positions.Num() - 2]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert2 = Positions[Positions.Num() - 3] + (Positions[Positions.Num() - 1] - Positions[Positions.Num() - 3]).GetSafeNormal() * Tolerance;

						const FVector& SmallTriVertWS0 = GetActorTransform().TransformPosition(SmallTriVert0);
						const FVector& SmallTriVertWS1 = GetActorTransform().TransformPosition(SmallTriVert1);
						const FVector& SmallTriVertWS2 = GetActorTransform().TransformPosition(SmallTriVert2);

						bool bIntersectionExist = false;
						for (UPrimitiveComponent* Primitive : OverlapPrimitives)
						{
							FHitResult HitResult;
							bool bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS0, SmallTriVertWS1, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS0, bHit ? HitResult.Location : SmallTriVertWS1, FColor(255, 128, 0), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS1, FColor(0, 128, 255), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS1, SmallTriVertWS2, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS1, bHit ? HitResult.Location : SmallTriVertWS2, FColor(255, 128, 0), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS2, FColor(0, 128, 255), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS2, SmallTriVertWS0, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS2, bHit ? HitResult.Location : SmallTriVertWS0, FColor(255, 128, 0), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS0, FColor(0, 128, 255), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}
						}

						if (!bIntersectionExist)
						{
							int32 LastIdx = Positions.Num() - 2;
							PrevPositions.RemoveAt(LastIdx);
							Positions.RemoveAt(LastIdx);
							EdgeIdxOfPositions.RemoveAt(LastIdx);
							MovedFlagOfPositions.RemoveAt(LastIdx);

							bExistAddedParticle = false;
						}
					}

					if (!bExistAddedParticle)
					{
						PrevPositions[Positions.Num() - 1] = Positions[Positions.Num() - 1];
					}
				}
				//
				// 終点セグメントのCollisionPhase　ここまで
				// 

				//
				// 始点終点以外のセグメントのCollisionPhase　ここから
				// 
				else
				{
					// エッジ端に到達した場合
					// 頂点周辺のエッジの状況を調査し、エッジ移動、頂点追加/削除の判定を行う
					// TODO:今まで削除は始点と端点側のブロックでTriangleの3つのエッジとPrimitiveの交差判定でやってきたが本来はここでやるべき

					// 頂点を中心としたTolerance半径の球と接触するエッジを収集する
					TArray<int32> IntersectedEdgeIndices;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& EdgeStart = Edge.Key;
						const FVector& EdgeEnd = Edge.Value;

						double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(Positions[ParticleIdx], EdgeStart, EdgeEnd);
						if (EdgeDistanceSq < ToleranceSquared)
						{
							IntersectedEdgeIndices.Add(EdgeIdx);
						}
					}

					if (IntersectedEdgeIndices.Num() == 1)
					{
						// 検出したエッジは現在所属しているエッジであるはず
						check(IntersectedEdgeIndices[0] == EdgeIdxOfPositions[ParticleIdx]);
						// メッシュとしてエッジ端に他のエッジが全く接してないのは許容できない
						check(false);
					}
					else if (IntersectedEdgeIndices.Num() > 1)
					{
						// エッジのペア配列を作る
						TArray<TPair<int32, int32>> EdgePairs;
						EdgePairs.SetNum(IntersectedEdgeIndices.Num() * (IntersectedEdgeIndices.Num() - 1) / 2); // n_C_2

						for (int32 EdgeIdx = 0, PairIdx = 0; EdgeIdx < IntersectedEdgeIndices.Num() - 1; EdgeIdx++)
						{
							for (int32 AnotherEdgeIdx = EdgeIdx + 1; AnotherEdgeIdx < IntersectedEdgeIndices.Num(); AnotherEdgeIdx++)
							{
								EdgePairs[PairIdx] = TPair<int32, int32>(IntersectedEdgeIndices[EdgeIdx], IntersectedEdgeIndices[AnotherEdgeIdx]);
								PairIdx++;
							}
						}

						// TODO:頂点間がToleranceより近い場合は頂点をまとめるべきなのでこのTolerance処理は必要ない
						const FVector& PreSegmentDir = (Positions[ParticleIdx] - Positions[ParticleIdx - 1]).GetSafeNormal(Tolerance, FVector::ZAxisVector); // ZAxisVectorにしているのはデフォルトのZeroVectorになると内積判定がおかしくなるのでとりあえず
						const FVector& PostSegmentDir = (Positions[ParticleIdx + 1] - Positions[ParticleIdx]).GetSafeNormal(Tolerance, FVector::ZAxisVector); // ZAxisVectorにしているのはデフォルトのZeroVectorになると内積判定がおかしくなるのでとりあえず
						double TwoSegmentDotProduct = FVector::DotProduct(PreSegmentDir, PostSegmentDir);

						FVector ParticleNormal = FVector::ZAxisVector;
						if (FVector::Coincident(PreSegmentDir, PostSegmentDir)) //TODO:ToleranceはFVector::Coincidentのデフォルト任せ
						{
							// 前セグメントと平行でないセグメントを探してそれを採用
							// TODO:後ろ方向にしか探してないので偏りがある
							for (int32 PostParticleIdx = ParticleIdx + 1; PostParticleIdx < Positions.Num(); PostParticleIdx++)
							{
								// TODO:頂点間がToleranceより近い場合は頂点をまとめるべきなのでこのTolerance処理は必要ない
								const FVector& NextDir = (Positions[PostParticleIdx] - Positions[ParticleIdx]).GetSafeNormal(Tolerance, FVector::ZAxisVector); // ZAxisVectorにしているのはデフォルトのZeroVectorになると内積判定がおかしくなるのでとりあえず
								double NextDotProduct = FVector::DotProduct(PreSegmentDir, NextDir);
								if (FMath::Abs(NextDotProduct) < 1.0f - Tolerance)
								{
									// 頂点位置から、前頂点と後頂点を通る直線に下ろした垂線の足
									const FVector& DropFoot = FMath::ClosestPointOnInfiniteLine(Positions[ParticleIdx - 1], Positions[PostParticleIdx], Positions[ParticleIdx]);
									ParticleNormal = (Positions[ParticleIdx] - DropFoot).GetSafeNormal();
									break;
								}
							}
						}
						else if (FVector::Coincident(-PreSegmentDir, PostSegmentDir)) //TODO:ToleranceはFVector::Coincidentのデフォルト任せ
						{
							
							ParticleNormal = PreSegmentDir;
						}
						else
						{
							// 頂点位置から、前頂点と後頂点を通る直線に下ろした垂線の足
							const FVector& DropFoot = FMath::ClosestPointOnInfiniteLine(Positions[ParticleIdx - 1], Positions[ParticleIdx + 1], Positions[ParticleIdx]);
							ParticleNormal = (Positions[ParticleIdx] - DropFoot).GetSafeNormal();
						}

						enum class CornerType : uint8
						{
							InnerCorner = 0,
							UnstableCorner,
							SideEdge,
							OuterCorner,
						};

						// エッジペアごとのコーナータイプ情報
						TArray<CornerType> CornerTypes;
						CornerTypes.SetNum(EdgePairs.Num());

						// エッジペアごとのコーナータイプ情報に付加するエッジインデックス情報
						TArray<int32> CornerEdgeIdxInfos;
						CornerEdgeIdxInfos.SetNum(EdgePairs.Num());

						// TODO:後で判定必要？
						//check(ParticleNormal != FVector::ZAxisVector);

						// エッジペアごとに頂点との関係性テーブルを作成
						for (int32 PairIdx = 0; PairIdx < EdgePairs.Num(); PairIdx++)
						{
							const TPair<int32, int32>& EdgePair = EdgePairs[PairIdx];
							int32 EdgeIdxA = EdgePair.Key;
							int32 EdgeIdxB = EdgePair.Value;

							TPair<FVector, FVector> EdgeA = RopeBlockerTriMeshEdgeArray[EdgeIdxA];
							TPair<FVector, FVector> EdgeB = RopeBlockerTriMeshEdgeArray[EdgeIdxB];

							// エッジは交点（ここでは頂点が交点のTolerance範囲の近傍んあるとして頂点で代用）から伸びる方向を前方向としておく
							// Valueの方が交点から遠い方とする
							if ((EdgeA.Key - Positions[ParticleIdx]).SizeSquared() > (EdgeA.Value - Positions[ParticleIdx]).SizeSquared())
							{
								FVector SwapTmp = EdgeA.Value;
								EdgeA.Value = EdgeA.Key;
								EdgeA.Key = SwapTmp;
							}
							if ((EdgeB.Key - Positions[ParticleIdx]).SizeSquared() > (EdgeB.Value - Positions[ParticleIdx]).SizeSquared())
							{
								FVector SwapTmp = EdgeB.Value;
								EdgeB.Value = EdgeB.Key;
								EdgeB.Key = SwapTmp;
							}

							// TODO:頂点間がToleranceより近い場合は頂点をまとめるべきなのでこのTolerance処理は必要ない
							const FVector& EdgeADir = (EdgeA.Value - EdgeA.Key).GetSafeNormal(Tolerance, FVector::ZAxisVector); // ZAxisVectorにしているのはデフォルトのZeroVectorになると外積判定がおかしくなるのでとりあえず
							const FVector& EdgeBDir = (EdgeB.Value - EdgeB.Key).GetSafeNormal(Tolerance, FVector::ZAxisVector); // ZAxisVectorにしているのはデフォルトのZeroVectorになると外積判定がおかしくなるのでとりあえず
							const FVector& Movement = Positions[ParticleIdx] - PrevPositions[ParticleIdx];

							if (FVector::Coincident(EdgeADir, EdgeBDir)) // エッジが0度。閉じたくさび。//TODO:ToleranceはFVector::Coincidentのデフォルト任せ
							{
								CornerTypes[PairIdx] = CornerType::InnerCorner;
								// TODO:Stableの場合、所属エッジがイテレーションごとに入れ替わる可能性があり、不安定になりそうだが？

								if (FVector::DotProduct(Movement, EdgeADir) > FVector::DotProduct(Movement, EdgeBDir))
								{
									// MoveAlongA
									CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
								}
								else
								{
									// MoveAlongB
									CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
								}
							}
							else if (FVector::Coincident(-EdgeADir, EdgeBDir)) // エッジが180度。//TODO:ToleranceはFVector::Coincidentのデフォルト任せ
							{
								// InnerCornerの別のエッジがあったときにそちらを優先したいのでInnerCornerでなくUnstableCornerにしておく
								CornerTypes[PairIdx] = CornerType::UnstableCorner;

								// TODO:実装が冗長
								if (FVector::DotProduct(Movement, EdgeADir) > FVector::DotProduct(Movement, EdgeBDir))
								{
									// MoveAlongA
									CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
								}
								else
								{
									// MoveAlongB
									CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
								}
							}
							else
							{
								const FVector& TwoEdgePlaneNormal = FVector::CrossProduct(EdgeADir, EdgeBDir).GetSafeNormal(Tolerance, FVector::ZAxisVector); // ZAxisVectorにしているのはデフォルトのZeroVectorになると外積判定がおかしくなるのでとりあえず;

								// 前セグメントと後セグメントが、エッジAとエッジBがなす平面を、両方表あるいは裏から
								// 貫通しているか、表裏逆方向に貫通しているかを判定する
								double DotProductPre = FVector::DotProduct(PreSegmentDir, TwoEdgePlaneNormal);
								double DotProductPost = FVector::DotProduct(PostSegmentDir, TwoEdgePlaneNormal);
								if ((DotProductPre > 0 && DotProductPost > 0)
									|| (DotProductPre < 0 && DotProductPost < 0)) // 両方表あるいは両方裏から貫通している
								{
									const FVector& ParticleNormalProjected = FVector::VectorPlaneProject(ParticleNormal, TwoEdgePlaneNormal);

									const FVector& CrossProductWithEdgeA = FVector::CrossProduct(EdgeADir, ParticleNormalProjected);
									const FVector& CrossProductWithEdgeB = FVector::CrossProduct(ParticleNormalProjected, EdgeBDir);

									// 頂点での折れ曲がりの法線がエッジAとエッジBの区分する4領域のどちら方向にあるかをチェックする
									double DotProductA = FVector::DotProduct(CrossProductWithEdgeA, TwoEdgePlaneNormal);
									double DotProductB = FVector::DotProduct(CrossProductWithEdgeB, TwoEdgePlaneNormal);

									if (DotProductA > 0)
									{
										if (DotProductB > 0)
										{
											CornerTypes[PairIdx] = CornerType::InnerCorner;
											// TODO:InnerCornerの場合、所属エッジがイテレーションごとに入れ替わる可能性があり、不安定になりそうだが？MoveAlongでなくStableという状態を作ってエッジ移動させずに安定状態として扱うべき？

											// TODO:実装が冗長
											if (FVector::DotProduct(Movement, EdgeADir) > FVector::DotProduct(Movement, EdgeBDir))
											{
												// MoveAlongA
												CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
											}
											else
											{
												// MoveAlongB
												CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
											}
										}
										else // DotProductB <= 0
										{
											CornerTypes[PairIdx] = CornerType::SideEdge;
											// IgnoreB
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
									}
									else // DotProductA <= 0
									{
										if (DotProductB > 0)
										{
											CornerTypes[PairIdx] = CornerType::SideEdge;
											// IgnoreA
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
										else // DotProductB <= 0
										{
											CornerTypes[PairIdx] = CornerType::UnstableCorner;
											// TODO:これはMovementPhaseでの動きから判定が必要

											// TODO:実装が冗長
											if (FVector::DotProduct(Movement, EdgeADir) > FVector::DotProduct(Movement, EdgeBDir))
											{
												// MoveAlongA
												CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
											}
											else
											{
												// MoveAlongB
												CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
											}
										}
									}
								}
								else // 表裏逆方向に貫通している。あるいは片方が平面上にある
								{
									//TODO: OuterCornerのくさびから交点に収束して2頂点をマージするケースも実装してない
									const FVector& PreSegmentProjected = FVector::VectorPlaneProject(PreSegmentDir, TwoEdgePlaneNormal);

									// PreSegmentは交点に向かう方向なのでわかりやすいように交点から出ていく方向になるようにマイナスをかける
									const FVector& PreSegmentCrossProductWithEdgeA = FVector::CrossProduct(EdgeADir, -PreSegmentProjected);
									const FVector& PreSegmentCrossProductWithEdgeB = FVector::CrossProduct(-PreSegmentProjected, EdgeBDir);

									// 前のセグメントがエッジAとエッジBのなす平面の4領域のどれを通るかを判定する内積
									double PreSegmentDotProductA = FVector::DotProduct(PreSegmentCrossProductWithEdgeA, TwoEdgePlaneNormal);
									double PreSegmentDotProductB = FVector::DotProduct(PreSegmentCrossProductWithEdgeB, TwoEdgePlaneNormal);

									const FVector& PostSegmentProjected = FVector::VectorPlaneProject(PostSegmentDir, TwoEdgePlaneNormal);

									const FVector& PostSegmentCrossProductWithEdgeA = FVector::CrossProduct(EdgeADir, PostSegmentProjected);
									const FVector& PostSegmentCrossProductWithEdgeB = FVector::CrossProduct(PostSegmentProjected, EdgeBDir);

									// 前のセグメントがエッジAとエッジBのなす平面の4領域のどれを通るかを判定する内積
									double PostSegmentDotProductA = FVector::DotProduct(PostSegmentCrossProductWithEdgeA, TwoEdgePlaneNormal);
									double PostSegmentDotProductB = FVector::DotProduct(PostSegmentCrossProductWithEdgeB, TwoEdgePlaneNormal);

									if (PreSegmentDotProductA > 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB > 0)
									{
										CornerTypes[PairIdx] = CornerType::OuterCorner;
										// CrossesAThenB
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
									}
									else if (PreSegmentDotProductA < 0 && PreSegmentDotProductB > 0 && PostSegmentDotProductA > 0 && PostSegmentDotProductB < 0)
									{
										CornerTypes[PairIdx] = CornerType::OuterCorner;
										// CrossesBThenA
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
									}
									else if (PreSegmentDotProductA < 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA > 0 && PostSegmentDotProductB > 0)
									{
										// TODO:実装。確かInnerConer扱いだが忘れた。動画見て確認する
										CornerTypes[PairIdx] = CornerType::InnerCorner;
										// TODO:とりあえず
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
									}
									else if (PreSegmentDotProductA > 0 && PreSegmentDotProductB > 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB < 0)
									{
										// TODO:実装。確かInnerConer扱いだが忘れた。動画見て確認する
										CornerTypes[PairIdx] = CornerType::InnerCorner;
										// TODO:とりあえず
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
									}
									else
									{
										// TODO:表裏貫通でこのケースは存在しない？動画見て確認する
										CornerTypes[PairIdx] = CornerType::InnerCorner;
										// TODO:とりあえず
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
									}
								}
							}
						}

						// エッジ移動に採用するペアの候補を洗い出す。まずは現在のエッジを含むペアを抽出する。
						check(EdgeIdxOfPositions[ParticleIdx] != INDEX_NONE);
						TArray<int32> CandidatesPairIndices;
						for (int32 PairIdx = 0; PairIdx < EdgePairs.Num(); PairIdx++)
						{
							const TPair<int32, int32>& EdgePair = EdgePairs[PairIdx];
							int32 EdgeIdxA = EdgePair.Key;
							int32 EdgeIdxB = EdgePair.Value;
							if ((EdgeIdxA == EdgeIdxOfPositions[ParticleIdx] || EdgeIdxB == EdgeIdxOfPositions[ParticleIdx])
								&& CornerTypes[PairIdx] != CornerType::SideEdge) // SideEdgeは移動候補から除外する
							{
								CandidatesPairIndices.Add(PairIdx);
							}
						}

						// 現在のエッジを含むペアのテーブルから、エッジ移動を決定する
						if (CandidatesPairIndices.Num() > 0) // 移動候補が全くなければ移動しない
						{
							// ペアをCornerTypeごとの配列に分類
							TArray<int32> InnerCornerPairIndices;
							TArray<int32> UnstableCornerPairIndices;
							TArray<int32> OuterCornerPairIndices;

							for (int32 PairIdx : CandidatesPairIndices)
							{
								switch (CornerTypes[PairIdx])
								{
									case CornerType::InnerCorner:
										InnerCornerPairIndices.Add(PairIdx);
										break;
									case CornerType::UnstableCorner:
										UnstableCornerPairIndices.Add(PairIdx);
										break;
									case CornerType::OuterCorner:
										OuterCornerPairIndices.Add(PairIdx);
										break;
									case CornerType::SideEdge:
									default:
										check(false);
										break;
								}
							}

							if (InnerCornerPairIndices.Num() > 0)
							{
								// TODO:複数あったらどれを採用する？
								// TODO:とりあえず一個目のものを採用しておく
								const TPair<int32, int32>& EdgePair = EdgePairs[InnerCornerPairIndices[0]];
								// ペアの中で今のエッジじゃない方にエッジ移動する
								// TODO:CornerEdgeIdxInfos[PairIdx]は使わない？
								if (EdgePair.Key != EdgeIdxOfPositions[ParticleIdx])
								{
									EdgeIdxOfPositions[ParticleIdx] = EdgePair.Key;
									// ここでMovelFlagを立てるとイテレーションのたびにエッジが交互に移動してイテレーションが
									// 無限ループになるので立てない
									// TODO:本来はMoveAlongだけでなくStable状態も作るべき
									//MovedFlagOfPositions[ParticleIdx] = true;
								}
								else if (EdgePair.Value != EdgeIdxOfPositions[ParticleIdx])
								{
									EdgeIdxOfPositions[ParticleIdx] = EdgePair.Value;
									// ここでMovelFlagを立てるとイテレーションのたびにエッジが交互に移動してイテレーションが
									// 無限ループになるので立てない
									// TODO:本来はMoveAlongだけでなくStable状態も作るべき
									//MovedFlagOfPositions[ParticleIdx] = true;
								}
								else
								{
									check(false);
								}
							}
							else if (UnstableCornerPairIndices.Num() > 0)
							{
								// TODO:複数あったらどれを採用する？
							}
							else if (OuterCornerPairIndices.Num() > 0)
							{
								// TODO:これってどうするんだ？
							}
						}
					}
				}
				//
				// 始点終点以外のセグメントのCollisionPhase　ここまで
				// 
			}
			//
			// CollisionPhase　ここまで
			//

			else // bNeedCollisionPhaseがflaseのときの処理
			{
				check(!bExistAddedParticle);
				PrevPositions[ParticleIdx] = Positions[ParticleIdx];
			}
		}
		//
		// パーティクルループ　ここまで
		//

		//
		// イテレーション早期打ち切りのための動いた頂点があるかどうかの判定
		//
		bExistMovedParticle = false;
		for (int32 ParticleIdx = 0; ParticleIdx < Positions.Num() - 1; ParticleIdx++)
		{
			if (MovedFlagOfPositions[ParticleIdx])
			{
				bExistMovedParticle = true;
				break;
			}
		}
	}
	//
	// イテレーションループ　ここまで
	//
}

void ATautRopeSimulatorCPU::SolveRopeBlockersCollisionConstraint()
{
	using namespace NiagaraSandbox::RopeSimulator;

	// MovementPhaseとCollisionPhaseの全頂点のイテレーション。収束するまでループする。
	bool bExistMovedParticle = false;
	if (Positions[0] != PrevPositions[0]
		|| Positions[Positions.Num() - 1] != PrevPositions[Positions.Num() - 1]) // TODO:Toleranceを入れないでみる
	{
		bExistMovedParticle = true;
	}

	//
	// イテレーションループ　ここから
	//
	for (int32 IterCount = 0; IterCount < MaxIteration && bExistMovedParticle; IterCount++)
	{
		bool bParticleIdxLoopAgain = false;

		//
		// パーティクルループ　ここから
		//
		// TODO: 逆方向ループはあとで必要か検討する
		for (int32 ParticleIdx = 0; ParticleIdx < Positions.Num(); ParticleIdx = (bParticleIdxLoopAgain && (ParticleIdx != Positions.Num() - 1)) ? ParticleIdx : ParticleIdx + 1) // 終点だったとき以外、頂点追加があったらもう一度そこから行う
		{
			bool bNeedCollisionPhase = false;
			MovedFlagOfPositions[ParticleIdx] = false;

			//
			// MovementPhase　ここから
			//
			{
				if (ParticleIdx == 0)
				{
					bNeedCollisionPhase = (Positions[0] != PrevPositions[0]); // TODO:Toleranceを入れないでみる

					// 始点と終点は1フレームに一度しか動かさないので即Movedフラグを下げる
					MovedFlagOfPositions[ParticleIdx] = false;
				}
				else if (ParticleIdx == (Positions.Num() - 1))
				{
					bNeedCollisionPhase = (Positions[Positions.Num() - 1] != PrevPositions[Positions.Num() - 1]); // TODO:Toleranceを入れないでみる

					// 始点と終点は1フレームに一度しか動かさないので即Movedフラグを下げる
					MovedFlagOfPositions[ParticleIdx] = false;
				}
				else // 始点と終点以外は最短コンストレイント
				{
					check(EdgeIdxOfPositions[ParticleIdx] != INDEX_NONE);
					const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdxOfPositions[ParticleIdx]];
					const FVector& EdgeStart = Edge.Key;
					const FVector& EdgeEnd = Edge.Value;

					FVector StartPoint = Positions[ParticleIdx - 1];
					FVector EndPoint = Positions[ParticleIdx + 1];

					// 頂点の位置が一致していると最短コンストレイントでは動かないので隣の頂点を使う
					for (int32 i = ParticleIdx - 1; i >= 0; i--)
					{
						if ((Positions[ParticleIdx] - Positions[i]).SizeSquared() > ToleranceSquared)
						{
							StartPoint = Positions[i];
							break;
						}
					}
					for (int32 i = ParticleIdx + 1; i < Positions.Num(); i++)
					{
						if ((Positions[ParticleIdx] - Positions[i]).SizeSquared() > ToleranceSquared)
						{
							EndPoint = Positions[i];
							break;
						}
					}

					// 動いてるのがStartPointであろうとEndPointであろうとStartPointPlane側を回転させて
					// 最短交点を計算する

					// エッジの直線に垂線を下ろした点をエッジを直線にして計算
					const FVector& StartPointDropFoot = FMath::ClosestPointOnInfiniteLine(EdgeStart, EdgeEnd, StartPoint);
					const FVector& EndPointDropFoot = FMath::ClosestPointOnInfiniteLine(EdgeStart, EdgeEnd, EndPoint);

					double StartPointPerpLen = (StartPoint - StartPointDropFoot).Size();
					double EndPointPerpLen = (EndPoint - EndPointDropFoot).Size();
					
					// 最短になる交点は垂線の長さの比による線形補間で決まる
					double Alpha = StartPointPerpLen / (StartPointPerpLen + EndPointPerpLen);
					const FVector& ShortestPointOnInfiniteLine = FMath::Lerp(StartPointDropFoot, EndPointDropFoot, Alpha);
					double EdgeLenSqr = FVector::DotProduct(EdgeEnd - EdgeStart, EdgeEnd - EdgeStart);

					// エッジは線分なのでクランプする
					bool bClamped = false;
					if (FVector::DotProduct(ShortestPointOnInfiniteLine - EdgeStart, EdgeEnd - EdgeStart) > (EdgeLenSqr - ToleranceSquared))
					{
						// EdgeEndぴったりにはせず、Tolerance/2離れた場所にする。ぴったりにすると頂点マージが
						// 必要な時に2頂点が同じ場所になりえるので判定に問題が出る。
						// 交差点検出ができるようにToleranceでなくTolerance/2にしている
						Positions[ParticleIdx] = EdgeEnd - (EdgeEnd - EdgeStart).GetSafeNormal() * Tolerance * 0.5;
						bClamped = true;
					}
					else if (FVector::DotProduct(ShortestPointOnInfiniteLine - EdgeEnd, EdgeStart - EdgeEnd) > (EdgeLenSqr - ToleranceSquared))
					{
						// EdgeEndぴったりにはせず、Tolerance/2離れた場所にする。ぴったりにすると頂点マージが
						// 必要な時に2頂点が同じ場所になりえるので判定に問題が出る。
						// 交差点検出ができるようにToleranceでなくTolerance/2にしている
						Positions[ParticleIdx] = EdgeStart + (EdgeEnd - EdgeStart).GetSafeNormal() * Tolerance * 0.5;
						bClamped = true;
					}
					else
					{
						Positions[ParticleIdx] = ShortestPointOnInfiniteLine;
						bClamped = false;
					}

					// エッジの端にクランプされたとき、動いていたらCollisionPhaseへ。属すべきエッジの変化が起きうるので、エッジ移動、頂点削除、追加などの判定を行う。
					bNeedCollisionPhase = bClamped && (Positions[ParticleIdx] != PrevPositions[ParticleIdx]); // TODO:Toleranceを入れないでみる

					// 収束のため閾値つきで動いたかどうか判定
					MovedFlagOfPositions[ParticleIdx] = ((PrevPositions[ParticleIdx] - Positions[ParticleIdx]).SizeSquared() > ToleranceSquared);
				}
			}
			//
			// MovementPhase　ここまで
			//

			// 追加頂点があると始点終点はその延長までしか動かさず再度MovementPhaseをやるので延長位置を保存する
			bParticleIdxLoopAgain = false;

			//
			// CollisionPhase　ここから
			//
			if (bNeedCollisionPhase)
			{
				//
				// 始点セグメントのCollisionPhase　ここから
				// 
				// 単に動いた時の前後セグメントでの頂点追加
				// TODO:エッジ移動とエッジに沿った削除はあとで実装する
				// TODO:現状、頂点追加は始点と終点のセグメント以外では考えない
				if (ParticleIdx == 0)
				{
					// 動いている頂点
					const FVector& TriVert0 = PrevPositions[ParticleIdx];
					const FVector& TriVert1 = Positions[ParticleIdx];
					const FVector& TriVert2 = PrevPositions[ParticleIdx + 1];

					double NearestEdgeDistanceSq = DBL_MAX;
					FVector NearestIntersectPoint = FVector::ZeroVector;
					int32 NearestEdgeIdx = INDEX_NONE;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						// 異なるエッジでないものは採用しない
						if (EdgeIdxOfPositions[ParticleIdx] == EdgeIdx || EdgeIdxOfPositions[ParticleIdx + 1] == EdgeIdx)
						{
							continue;
						}

						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& RayStart = Edge.Key;
						const FVector& RayEnd = Edge.Value;
						FVector IntersectPoint;
						FVector IntersectNormal;
						bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
						if (bIntersecting)
						{
							// 既存の動いてない方の頂点と近すぎるものは追加しない。複数エッジの交点と衝突した場合に複数頂点追加しうるので
							if ((IntersectPoint - TriVert2).SizeSquared() > ToleranceSquared)
							{
								// 元の線分と最も近いエッジを採用
								double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(IntersectPoint, TriVert0, TriVert2);
								if (EdgeDistanceSq < NearestEdgeDistanceSq)
								{
									// 等距離なら最も若いインデックスを採用する
									NearestEdgeDistanceSq = EdgeDistanceSq;
									NearestIntersectPoint = IntersectPoint;
									NearestEdgeIdx = EdgeIdx;
								}
							}
						}
					}

					// 頂点追加
					if (NearestEdgeIdx != INDEX_NONE)
					{
						// TODO:GDC動画のように延長線上を新たな終点におくと、同一平面に複数エッジがある
						// シェイプだと、同一平面上のエッジを拾い損ねることがFMath::SegmentTriangleIntersection()では簡単に起きる。
						// Triangleの辺にエッジがある場合に交差を検出失敗する。
						// よってPrevPositionsの更新は頂点の追加が終わるまでしない
						PrevPositions.Insert(NearestIntersectPoint, ParticleIdx + 1);
						Positions.Insert(NearestIntersectPoint, ParticleIdx + 1);
						EdgeIdxOfPositions.Insert(NearestEdgeIdx, ParticleIdx + 1);
						MovedFlagOfPositions.Insert(true, ParticleIdx + 1); // ここでtrueにしてもパーティクルループの自分の番のMovementPhaseで別途判定される

						bParticleIdxLoopAgain = true;
					}

					// エッジからはがす削除
					// TODO:終点と処理が冗長
					if (Positions.Num() >= 3)
					{
						// 0-2の線分の削除だけで判定していないのは、なにかに1をひっかけている状態で
						// 0-2を大きく動かすと0-2線分上にコリジョンがいない状態になり削除対象になってしまうから
						// TODO:本当は3つのライントレースでなくTriangleとコリジョンのOverlapをとりたかったがAPIがなかった
						// TODO:もっといいやり方あるかも

						bool bTraceComplex = false;
						FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(SolveRopeBlockersCollisionConstraint), bTraceComplex);
						// Toleranceだけ0と2を双方から縮めるのは、どちらかあるいは両方がエッジと接触
						// していたらそのエッジとヒット判定になり、1の削除判定をしたいのにできないため
						// TODO:もっといい方法ある？
						// Toleranceだけ1を0-2の線分側に近づけるのは、1がエッジと接触してると接触判定し続けるため
						// TODO:もっといい方法ある？

						// 2があまりに1に近いとTriangleにならずに正しい判定ができないのでそういうときは近くないものを探しに行く
						FVector Position2 = Positions[2];
						for (int32 i = 2; i < Positions.Num(); i++)
						{
							if ((Positions[i] - Positions[1]).SizeSquared() > ToleranceSquared)
							{
								Position2 = Positions[i];
								break;
							}
						}

						const FVector& SmallTriVert0 = Position2 + (Positions[0] - Position2).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert1 = Positions[1] + ((Position2 + Positions[0]) * 0.5 - Positions[1]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert2 = Positions[0] + (Position2 - Positions[0]).GetSafeNormal() * Tolerance;

						const FVector& SmallTriVertWS0 = GetActorTransform().TransformPosition(SmallTriVert0);
						const FVector& SmallTriVertWS1 = GetActorTransform().TransformPosition(SmallTriVert1);
						const FVector& SmallTriVertWS2 = GetActorTransform().TransformPosition(SmallTriVert2);

						bool bIntersectionExist = false;
						for (UPrimitiveComponent* Primitive : OverlapPrimitives)
						{
							FHitResult HitResult;
							bool bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS0, SmallTriVertWS1, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS0, bHit ? HitResult.Location : SmallTriVertWS1, FColor(255, 128, 0), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS1, FColor(0, 128, 255), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS1, SmallTriVertWS2, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS1, bHit ? HitResult.Location : SmallTriVertWS2, FColor(255, 128, 0), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS2, FColor(0, 128, 255), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS2, SmallTriVertWS0, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS2, bHit ? HitResult.Location : SmallTriVertWS0, FColor(255, 128, 0), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS0, FColor(0, 128, 255), false, -1.0f, ESceneDepthPriorityGroup::SDPG_World, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}
						}

						if (!bIntersectionExist)
						{
							PrevPositions.RemoveAt(1);
							Positions.RemoveAt(1);
							EdgeIdxOfPositions.RemoveAt(1);
							MovedFlagOfPositions.RemoveAt(1);

							// 追加したものが即削除された場合ももう追加判定はとめて次のパーティクルに進む。
							// 追加と即削除を繰り返すとParticleIdx=0のままでループが進まないので。
							bParticleIdxLoopAgain = false;
						}
					}

					if (!bParticleIdxLoopAgain)
					{
						PrevPositions[0] = Positions[0];
					}
				}
				//
				// 始点セグメントのCollisionPhase　ここまで
				// 

				//
				// 終点セグメントのCollisionPhase　ここから
				// 
				else if (ParticleIdx == (Positions.Num() - 1))
				{
					// 動いている頂点
					const FVector& TriVert1 = Positions[ParticleIdx];
					const FVector& TriVert2 = PrevPositions[ParticleIdx];
					
					// 固定している頂点は始点の判定と違ってPrevPositonsでなくPositionsなのに注意。
					// こうしないと1フレームでParticlesIdxの頂点が大きく動いたとき交差検出が漏れるケースがある
					// https://www.gdcvault.com/play/1027351/Rope-Simulation-in-Uncharted-4
					// の32分ごろの例。
					const FVector& TriVert0 = Positions[ParticleIdx - 1];

					// TODO: 上のifブロックと処理が冗長
					double NearestEdgeDistanceSq = DBL_MAX;
					FVector NearestIntersectPoint = FVector::ZeroVector;
					int32 NearestEdgeIdx = INDEX_NONE;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						// 異なるエッジでないものは採用しない
						if (EdgeIdxOfPositions[ParticleIdx - 1] == EdgeIdx || EdgeIdxOfPositions[ParticleIdx] == EdgeIdx)
						{
							continue;
						}

						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& RayStart = Edge.Key;
						const FVector& RayEnd = Edge.Value;
						FVector IntersectPoint;
						FVector IntersectNormal;
						bool bIntersecting = FMath::SegmentTriangleIntersection(RayStart, RayEnd, TriVert0, TriVert1, TriVert2, IntersectPoint, IntersectNormal);
						if (bIntersecting)
						{
							// 既存の動いてない方の頂点と近すぎるものは追加しない。複数エッジの交点と衝突した場合に複数頂点追加しうるので
							if ((IntersectPoint - TriVert0).SizeSquared() > ToleranceSquared)
							{
								// 元の線分と最も近いエッジを採用
								double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(IntersectPoint, TriVert0, TriVert2);
								if (EdgeDistanceSq < NearestEdgeDistanceSq)
								{
									// 等距離なら最も若いインデックスを採用する
									NearestEdgeDistanceSq = EdgeDistanceSq;
									NearestIntersectPoint = IntersectPoint;
									NearestEdgeIdx = EdgeIdx;
								}
							}
						}
					}

					if (NearestEdgeIdx != INDEX_NONE)
					{
						// TODO:GDC動画のように延長線上を新たな終点におくと、同一平面に複数エッジがある
						// シェイプだと、同一平面上のエッジを拾い損ねることがFMath::SegmentTriangleIntersection()では簡単に起きる。
						// Triangleの辺にエッジがある場合に交差を検出失敗する。
						// よってPrevPositionsの更新は頂点の追加が終わるまでしない
						PrevPositions.Insert(NearestIntersectPoint, ParticleIdx);
						Positions.Insert(NearestIntersectPoint, ParticleIdx);
						EdgeIdxOfPositions.Insert(NearestEdgeIdx, ParticleIdx);
						MovedFlagOfPositions.Insert(true, ParticleIdx); // ここでtrueにして次イテレーションで最短コンストレイントを行う // TODO:逆順ループを作れば次イテレーションにしなくてもよくなり収束も早くなるかも
						bParticleIdxLoopAgain = true;
					}

					// エッジからはがす削除
					// TODO:終点と処理が冗長
					if (Positions.Num() >= 3)
					{
						// 0-2の線分の削除だけで判定していないのは、なにかに1をひっかけている状態で
						// 0-2を大きく動かすと0-2線分上にコリジョンがいない状態になり削除対象になってしまうから
						// TODO:本当は3つのライントレースでなくTriangleとコリジョンのOverlapをとりたかったがAPIがなかった
						// TODO:もっといいやり方あるかも

						bool bTraceComplex = false;
						FCollisionQueryParams TraceParams(SCENE_QUERY_STAT(SolveRopeBlockersCollisionConstraint), bTraceComplex);
						// Toleranceだけ0と2を双方から縮めるのは、どちらかあるいは両方がエッジと接触
						// していたらそのエッジとヒット判定になり、1の削除判定をしたいのにできないため
						// TODO:もっといい方法ある？
						// Toleranceだけ1を0-2の線分側に近づけるのは、1がエッジと接触してると接触判定し続けるため
						// TODO:もっといい方法ある？

						// 2があまりに1に近いとTriangleにならずに正しい判定ができないのでそういうときは近くないものを探しに行く
						FVector Position2 = Positions[Positions.Num() - 3];
						for (int32 i = Positions.Num() - 3; i >= 0; i--)
						{
							if ((Positions[i] - Positions[Positions.Num() - 2]).SizeSquared() > ToleranceSquared)
							{
								Position2 = Positions[i];
								break;
							}
						}

						const FVector& SmallTriVert0 = Positions[Positions.Num() - 1] + (Position2 - Positions[Positions.Num() - 1]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert1 = Positions[Positions.Num() - 2] + ((Positions[Positions.Num() - 1] + Position2) * 0.5 - Positions[Positions.Num() - 2]).GetSafeNormal() * Tolerance;
						const FVector& SmallTriVert2 = Position2 + (Positions[Positions.Num() - 1] - Position2).GetSafeNormal() * Tolerance;

						const FVector& SmallTriVertWS0 = GetActorTransform().TransformPosition(SmallTriVert0);
						const FVector& SmallTriVertWS1 = GetActorTransform().TransformPosition(SmallTriVert1);
						const FVector& SmallTriVertWS2 = GetActorTransform().TransformPosition(SmallTriVert2);

						bool bIntersectionExist = false;
						for (UPrimitiveComponent* Primitive : OverlapPrimitives)
						{
							FHitResult HitResult;
							bool bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS0, SmallTriVertWS1, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS0, bHit ? HitResult.Location : SmallTriVertWS1, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS1, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS1, SmallTriVertWS2, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS1, bHit ? HitResult.Location : SmallTriVertWS2, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS2, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}

							bHit = Primitive->LineTraceComponent(HitResult, SmallTriVertWS2, SmallTriVertWS0, TraceParams);
							if (bDrawTraceToRemove)
							{
								// UPrimitiveComponent::K2_LineTraceComponent()を参考にしている
								DrawDebugLine(GetWorld(), SmallTriVertWS2, bHit ? HitResult.Location : SmallTriVertWS0, FColor(255, 128, 0), false, -1.0f, 0, 2.0f);
								if(bHit)
								{
									DrawDebugLine(GetWorld(), HitResult.Location, SmallTriVertWS0, FColor(0, 128, 255), false, -1.0f, 0, 2.0f);
								}
							}

							if (bHit)
							{
								bIntersectionExist = true;
								break;
							}
						}

						if (!bIntersectionExist)
						{
							int32 LastIdx = Positions.Num() - 2;
							PrevPositions.RemoveAt(LastIdx);
							Positions.RemoveAt(LastIdx);
							EdgeIdxOfPositions.RemoveAt(LastIdx);
							MovedFlagOfPositions.RemoveAt(LastIdx);

							bParticleIdxLoopAgain = false;
						}
					}

					if (!bParticleIdxLoopAgain)
					{
						PrevPositions[Positions.Num() - 1] = Positions[Positions.Num() - 1];
					}
				}
				//
				// 終点セグメントのCollisionPhase　ここまで
				// 

				//
				// 始点終点以外のセグメントのCollisionPhase　ここから
				// 
				else
				{
					// エッジ端に到達した場合
					// 頂点周辺のエッジの状況を調査し、エッジ移動、頂点追加/削除の判定を行う
					// TODO:今まで削除は始点と端点側のブロックでTriangleの3つのエッジとPrimitiveの交差判定でやってきたが本来はここでやるべき

					// 頂点を中心としたTolerance半径の球と接触するエッジを収集する
					TArray<int32> IntersectedEdgeIndices;
					for (int32 EdgeIdx = 0; EdgeIdx < RopeBlockerTriMeshEdgeArray.Num(); EdgeIdx++)
					{
						const TPair<FVector, FVector>& Edge = RopeBlockerTriMeshEdgeArray[EdgeIdx];
						const FVector& EdgeStart = Edge.Key;
						const FVector& EdgeEnd = Edge.Value;

						double EdgeDistanceSq = NiagaraSandbox::RopeSimulator::PointDistToSegmentSquared(Positions[ParticleIdx], EdgeStart, EdgeEnd);
						if (EdgeDistanceSq < ToleranceSquared)
						{
							IntersectedEdgeIndices.Add(EdgeIdx);
						}
					}

					check(IntersectedEdgeIndices.Num());
					if (IntersectedEdgeIndices.Num() == 1)
					{
						// 検出したエッジは現在所属しているエッジであるはず
						check(IntersectedEdgeIndices[0] == EdgeIdxOfPositions[ParticleIdx]);
						// メッシュとしてエッジ端に他のエッジが全く接してないのは許容できない
						check(false);
					}
					else if (IntersectedEdgeIndices.Num() > 1)
					{
						// エッジのペア配列を作る
						TArray<TPair<int32, int32>> EdgePairs;
						EdgePairs.SetNum(IntersectedEdgeIndices.Num() * (IntersectedEdgeIndices.Num() - 1) / 2); // n_C_2

						for (int32 EdgeIdx = 0, PairIdx = 0; EdgeIdx < IntersectedEdgeIndices.Num() - 1; EdgeIdx++)
						{
							for (int32 AnotherEdgeIdx = EdgeIdx + 1; AnotherEdgeIdx < IntersectedEdgeIndices.Num(); AnotherEdgeIdx++)
							{
								EdgePairs[PairIdx] = TPair<int32, int32>(IntersectedEdgeIndices[EdgeIdx], IntersectedEdgeIndices[AnotherEdgeIdx]);
								PairIdx++;
							}
						}

						// 頂点マージが発生するようなときは頂点間の距離がTolerance以下でも方向が正確に得られないと
						// 正確なエッジ移動先は求まらないのでここはTolerance以下のものを使う
						//const FVector& PreSegmentDir = (Positions[ParticleIdx] - Positions[ParticleIdx - 1]).GetSafeNormal(Tolerance);
						FVector PreSegmentDir = (Positions[ParticleIdx] - Positions[ParticleIdx - 1]);
						double PreSegmentDirLen = PreSegmentDir.Size();
						if (PreSegmentDirLen < UE_KINDA_SMALL_NUMBER)
						{
							PreSegmentDir = FVector::ZeroVector;
						}
						else
						{
							PreSegmentDir /= PreSegmentDirLen;
						}
						//const FVector& PostSegmentDir = (Positions[ParticleIdx + 1] - Positions[ParticleIdx]).GetSafeNormal(Tolerance);
						FVector PostSegmentDir = (Positions[ParticleIdx + 1] - Positions[ParticleIdx]);
						double PostSegmentDirLen = PostSegmentDir.Size();
						if (PostSegmentDirLen < UE_KINDA_SMALL_NUMBER)
						{
							PostSegmentDir = FVector::ZeroVector;
						}
						else
						{
							PostSegmentDir /= PostSegmentDirLen;
						}

						const FVector& TensionDir = (-PreSegmentDir + PostSegmentDir).GetSafeNormal(Tolerance);

						enum class CornerType : uint8
						{
							InnerCorner = 0,
							UnstableCorner,
							SideEdge,
							OuterCorner,
						};

						// エッジペアごとのコーナータイプ情報
						TArray<CornerType> CornerTypes;
						CornerTypes.SetNum(EdgePairs.Num());

						// エッジペアごとのコーナータイプ情報に付加するエッジインデックス情報
						TArray<int32> CornerEdgeIdxInfos;
						CornerEdgeIdxInfos.SetNum(EdgePairs.Num());

						// TODO:後で判定必要？
						//check(ParticleNormal != FVector::ZAxisVector);

						// エッジペアごとのコーナー情報テーブルを作成
						for (int32 PairIdx = 0; PairIdx < EdgePairs.Num(); PairIdx++)
						{
							const TPair<int32, int32>& EdgePair = EdgePairs[PairIdx];
							int32 EdgeIdxA = EdgePair.Key;
							int32 EdgeIdxB = EdgePair.Value;

							TPair<FVector, FVector> EdgeA = RopeBlockerTriMeshEdgeArray[EdgeIdxA];
							TPair<FVector, FVector> EdgeB = RopeBlockerTriMeshEdgeArray[EdgeIdxB];

							// エッジの片方が長さがToleranceより小さければSideEdgeにしてそちらをIgnoreにする
							if ((EdgeA.Key - EdgeA.Value).SizeSquared() < ToleranceSquared)
							{
								CornerTypes[PairIdx] = CornerType::SideEdge;
								// IgnoreA
								CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
								continue;
							}
							if ((EdgeB.Key - EdgeB.Value).SizeSquared() < ToleranceSquared)
							{
								CornerTypes[PairIdx] = CornerType::SideEdge;
								// IgnoreB
								CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
								continue;
							}

							// エッジは交点（ここでは頂点が交点のTolerance範囲の近傍んあるとして頂点で代用）から伸びる方向を前方向としておく
							// Valueの方が交点から遠い方とする
							if ((EdgeA.Key - Positions[ParticleIdx]).SizeSquared() > (EdgeA.Value - Positions[ParticleIdx]).SizeSquared())
							{
								FVector SwapTmp = EdgeA.Value;
								EdgeA.Value = EdgeA.Key;
								EdgeA.Key = SwapTmp;
							}
							if ((EdgeB.Key - Positions[ParticleIdx]).SizeSquared() > (EdgeB.Value - Positions[ParticleIdx]).SizeSquared())
							{
								FVector SwapTmp = EdgeB.Value;
								EdgeB.Value = EdgeB.Key;
								EdgeB.Key = SwapTmp;
							}

							const FVector& EdgeADir = (EdgeA.Value - EdgeA.Key).GetSafeNormal(Tolerance);
							const FVector& EdgeBDir = (EdgeB.Value - EdgeB.Key).GetSafeNormal(Tolerance);

							// エッジペア平面と張力方向から各エッジペアのコーナー情報を求める
							if (FVector::Coincident(EdgeADir, EdgeBDir)) // エッジが0度。閉じたくさび。//TODO:ToleranceはFVector::Coincidentのデフォルト任せ
							{
								CornerTypes[PairIdx] = CornerType::InnerCorner;
								// TODO:Stableの場合、所属エッジがイテレーションごとに入れ替わる可能性があり、不安定になりそうだが？

								// DotProductの差はごく小さいが、それで比較して振り分ける
								if (FVector::DotProduct(TensionDir, EdgeADir) > FVector::DotProduct(TensionDir, EdgeBDir))
								{
									// MoveAlongA or StableCorner
									CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
								}
								else
								{
									// MoveAlongB or StableCorner
									CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
								}
							}
							else if (FVector::Coincident(-EdgeADir, EdgeBDir)) // エッジが180度。//TODO:ToleranceはFVector::Coincidentのデフォルト任せ
							{
								// InnerCornerでもUnstableCornerでもいいのだが少なくともInnerCorner::StableCornerではないのでUnstableCornerにしておく
								CornerTypes[PairIdx] = CornerType::UnstableCorner;

								if (FVector::DotProduct(TensionDir, EdgeADir) > FVector::DotProduct(TensionDir, EdgeBDir))
								{
									// MoveAlongA
									CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
								}
								else
								{
									// MoveAlongB
									CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
								}
							}
							else // エッジペア平面が成立するケース
							{
								check((EdgeA.Key - EdgeA.Value).SizeSquared() >= ToleranceSquared);
								check((EdgeB.Key - EdgeB.Value).SizeSquared() >= ToleranceSquared);

								// GetUnsafeNormal()にしているが、EdgeADirとEdgeBDirが0なケースや平行なケースは分岐済み
								const FVector& TwoEdgePlaneNormal = FVector::CrossProduct(EdgeADir, EdgeBDir).GetUnsafeNormal();

								const FVector& TensionDirProjected = FVector::VectorPlaneProject(TensionDir, TwoEdgePlaneNormal);

								// 前セグメントと後セグメントが、エッジAとエッジBがなす平面を、両方表あるいは裏から
								// 貫通しているか、表裏逆方向に貫通しているかを判定する
								double DotProductPre = FVector::DotProduct(PreSegmentDir, TwoEdgePlaneNormal);
								double DotProductPost = FVector::DotProduct(PostSegmentDir, TwoEdgePlaneNormal);
								if ((DotProductPre > 0 && DotProductPost > 0)
									|| (DotProductPre < 0 && DotProductPost < 0)) // 両方表あるいは両方裏から貫通している
								{
									const FVector& CrossProductWithEdgeA = FVector::CrossProduct(TensionDirProjected, EdgeADir);
									const FVector& CrossProductWithEdgeB = FVector::CrossProduct(TensionDirProjected, EdgeBDir);

									// 頂点での折れ曲がりの法線がエッジAとエッジBの区分する4領域のどちら方向にあるかをチェックする
									double DotProductA = FVector::DotProduct(CrossProductWithEdgeA, TwoEdgePlaneNormal);
									double DotProductB = FVector::DotProduct(CrossProductWithEdgeB, TwoEdgePlaneNormal);

									if (DotProductA < 0)
									{
										if (DotProductB < 0)
										{
											CornerTypes[PairIdx] = CornerType::SideEdge;
											// IgnoreB
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
										else // DotProductB >= 0
										{
											CornerTypes[PairIdx] = CornerType::UnstableCorner;
											// TODO:これはMovementPhaseでの動きから判定が必要

											// TODO:実装が冗長
											if (FVector::DotProduct(TensionDirProjected, EdgeADir) > FVector::DotProduct(TensionDirProjected, EdgeBDir))
											{
												// MoveAlongA
												CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
											}
											else
											{
												// MoveAlongB
												CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
											}
										}
									}
									else // DotProductA >= 0
									{
										if (DotProductB < 0)
										{
											CornerTypes[PairIdx] = CornerType::InnerCorner;

											// TODO:実装が冗長
											if (FVector::DotProduct(TensionDirProjected, EdgeADir) > FVector::DotProduct(TensionDirProjected, EdgeBDir))
											{
												// MoveAlongA or StableCorner
												CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
											}
											else
											{
												// MoveAlongB or StableCorner
												CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
											}
										}
										else // DotProductB >= 0
										{
											CornerTypes[PairIdx] = CornerType::SideEdge;
											// IgnoreA
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
									}
								}
								else // 表裏逆方向に貫通している。あるいは片方が平面上にある
								{
									//TODO: OuterCornerのくさびから交点に収束して2頂点をマージするケースも実装してない
									// PreSegmentは交点に向かう方向なのでわかりやすいように交点から出ていく方向になるようにマイナスをかける
									const FVector& PreSegmentProjected = FVector::VectorPlaneProject(-PreSegmentDir, TwoEdgePlaneNormal);

									const FVector& PreSegmentCrossProductWithEdgeA = FVector::CrossProduct(PreSegmentProjected, EdgeADir);
									const FVector& PreSegmentCrossProductWithEdgeB = FVector::CrossProduct(PreSegmentProjected, EdgeBDir);

									// 前のセグメントがエッジAとエッジBのなす平面の4領域のどれを通るかを判定する内積
									double PreSegmentDotProductA = FVector::DotProduct(PreSegmentCrossProductWithEdgeA, TwoEdgePlaneNormal);
									double PreSegmentDotProductB = FVector::DotProduct(PreSegmentCrossProductWithEdgeB, TwoEdgePlaneNormal);

									const FVector& PostSegmentProjected = FVector::VectorPlaneProject(PostSegmentDir, TwoEdgePlaneNormal);

									const FVector& PostSegmentCrossProductWithEdgeA = FVector::CrossProduct(PostSegmentProjected, EdgeADir);
									const FVector& PostSegmentCrossProductWithEdgeB = FVector::CrossProduct(PostSegmentProjected, EdgeBDir);

									// 前のセグメントがエッジAとエッジBのなす平面の4領域のどれを通るかを判定する内積
									double PostSegmentDotProductA = FVector::DotProduct(PostSegmentCrossProductWithEdgeA, TwoEdgePlaneNormal);
									double PostSegmentDotProductB = FVector::DotProduct(PostSegmentCrossProductWithEdgeB, TwoEdgePlaneNormal);

									if (PreSegmentDotProductA < 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB < 0)
									{
										CornerTypes[PairIdx] = CornerType::UnstableCorner;
										// MoveAlongB
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
									}
									else if (PreSegmentDotProductA < 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB >= 0)
									{
										CornerTypes[PairIdx] = CornerType::UnstableCorner;
										// MoveAlongB
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
									}
									else if (PreSegmentDotProductA < 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA >= 0 && PostSegmentDotProductB < 0)
									{
										CornerTypes[PairIdx] = CornerType::InnerCorner;
										// TODO:実装が冗長
										if (FVector::DotProduct(TensionDirProjected, EdgeADir) >= FVector::DotProduct(TensionDirProjected, EdgeBDir))
										{
											// MoveAlongA or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
										else
										{
											// MoveAlongB or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
									}
									else if (PreSegmentDotProductA < 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA >= 0 && PostSegmentDotProductB >= 0)
									{
										CornerTypes[PairIdx] = CornerType::OuterCorner;
										// CrossesBThenA
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
									}

									else if (PreSegmentDotProductA < 0 && PreSegmentDotProductB >= 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB < 0)
									{
										CornerTypes[PairIdx] = CornerType::UnstableCorner;
										// MoveAlongB
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
									}
									else if (PreSegmentDotProductA < 0 && PreSegmentDotProductB >= 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB >= 0)
									{
										CornerTypes[PairIdx] = CornerType::UnstableCorner;
										// TODO:実装が冗長
										if (FVector::DotProduct(TensionDirProjected, EdgeADir) >= FVector::DotProduct(TensionDirProjected, EdgeBDir))
										{
											// MoveAlongA
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
										else
										{
											// MoveAlongB
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
									}
									else if (PreSegmentDotProductA < 0 && PreSegmentDotProductB >= 0 && PostSegmentDotProductA >= 0 && PostSegmentDotProductB < 0)
									{
										CornerTypes[PairIdx] = CornerType::InnerCorner;

										// TODO:実装が冗長
										if (FVector::DotProduct(TensionDirProjected, EdgeADir) >= FVector::DotProduct(TensionDirProjected, EdgeBDir))
										{
											// MoveAlongA or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
										else
										{
											// MoveAlongB or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
									}
									else if (PreSegmentDotProductA < 0 && PreSegmentDotProductB >= 0 && PostSegmentDotProductA >= 0 && PostSegmentDotProductB >= 0)
									{
										CornerTypes[PairIdx] = CornerType::UnstableCorner;
										// MoveAlongA
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
									}

									else if (PreSegmentDotProductA >= 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB < 0)
									{
										CornerTypes[PairIdx] = CornerType::InnerCorner;
										// TODO:実装が冗長
										if (FVector::DotProduct(TensionDirProjected, EdgeADir) >= FVector::DotProduct(TensionDirProjected, EdgeBDir))
										{
											// MoveAlongA or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
										else
										{
											// MoveAlongB or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
									}
									else if (PreSegmentDotProductA >= 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB >= 0)
									{
										CornerTypes[PairIdx] = CornerType::InnerCorner;
										// TODO:実装が冗長
										if (FVector::DotProduct(TensionDirProjected, EdgeADir) >= FVector::DotProduct(TensionDirProjected, EdgeBDir))
										{
											// MoveAlongA or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
										else
										{
											// MoveAlongB or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
									}
									else if (PreSegmentDotProductA >= 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA >= 0 && PostSegmentDotProductB < 0)
									{
										CornerTypes[PairIdx] = CornerType::InnerCorner;
										// TODO:実装が冗長
										if (FVector::DotProduct(TensionDirProjected, EdgeADir) >= FVector::DotProduct(TensionDirProjected, EdgeBDir))
										{
											// MoveAlongA or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
										else
										{
											// MoveAlongB or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
									}
									else if (PreSegmentDotProductA >= 0 && PreSegmentDotProductB < 0 && PostSegmentDotProductA >= 0 && PostSegmentDotProductB >= 0)
									{
										CornerTypes[PairIdx] = CornerType::InnerCorner;
										// TODO:実装が冗長
										if (FVector::DotProduct(TensionDirProjected, EdgeADir) >= FVector::DotProduct(TensionDirProjected, EdgeBDir))
										{
											// MoveAlongA or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
										else
										{
											// MoveAlongB or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
									}

									else if (PreSegmentDotProductA >= 0 && PreSegmentDotProductB >= 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB < 0)
									{
										CornerTypes[PairIdx] = CornerType::OuterCorner;
										// CrossesAThenB
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
									}
									else if (PreSegmentDotProductA >= 0 && PreSegmentDotProductB >= 0 && PostSegmentDotProductA < 0 && PostSegmentDotProductB >= 0)
									{
										CornerTypes[PairIdx] = CornerType::UnstableCorner;
										// MoveAlongA
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
									}
									else if (PreSegmentDotProductA >= 0 && PreSegmentDotProductB >= 0 && PostSegmentDotProductA >= 0 && PostSegmentDotProductB < 0)
									{
										CornerTypes[PairIdx] = CornerType::InnerCorner;
										// TODO:実装が冗長
										if (FVector::DotProduct(TensionDirProjected, EdgeADir) >= FVector::DotProduct(TensionDirProjected, EdgeBDir))
										{
											// MoveAlongA or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
										}
										else
										{
											// MoveAlongB or StableCorner
											CornerEdgeIdxInfos[PairIdx] = EdgeIdxB;
										}
									}
									else if (PreSegmentDotProductA >= 0 && PreSegmentDotProductB >= 0 && PostSegmentDotProductA >= 0 && PostSegmentDotProductB >= 0)
									{
										CornerTypes[PairIdx] = CornerType::UnstableCorner;
										// MoveAlongA
										CornerEdgeIdxInfos[PairIdx] = EdgeIdxA;
									}
									else
									{
										check(false);
									}
								}
							}
						} // コーナー情報テーブル作成ループ

						// コーナー情報作成テーブルからSideEdgeのIgnoreにいずれかの要素で一つでもなっているエッジを洗い出す
						TArray<int32> IgnoreIndices;

						for (int32 PairIdx = 0; PairIdx < EdgePairs.Num(); PairIdx++)
						{
							if (CornerTypes[PairIdx] == CornerType::SideEdge)
							{
								IgnoreIndices.AddUnique(CornerEdgeIdxInfos[PairIdx]);
							}
						}

						// コーナー情報作成テーブルでエッジがSideEdgeに全くなってないペアで、InnerCornerかOuterCornerのみを移動候補のペアとする
						TArray<int32> CandidatesPairIndices;
						for (int32 PairIdx = 0; PairIdx < EdgePairs.Num(); PairIdx++)
						{
							if (IgnoreIndices.Find(EdgePairs[PairIdx].Key) != INDEX_NONE)
							{
								continue;
							}

							if (IgnoreIndices.Find(EdgePairs[PairIdx].Value) != INDEX_NONE)
							{
								continue;
							}

							if (CornerTypes[PairIdx] == CornerType::InnerCorner || CornerTypes[PairIdx] == CornerType::UnstableCorner)
							{
								CandidatesPairIndices.Add(PairIdx);
							}
						}

						// 候補の中からMoveAlong先が張力的にもっともひっかかるエッジを移動先として採用。
						int32 MoveTargetEdgeIdx = INDEX_NONE;
						double MaxDotProduct = -DBL_MAX;
						for (int32 PairIdx : CandidatesPairIndices)
						{
							int32 MoveAlongEdgeIdx = CornerEdgeIdxInfos[PairIdx];
							TPair<FVector, FVector> CandidateEdge = RopeBlockerTriMeshEdgeArray[MoveAlongEdgeIdx];

							// エッジは交点（ここでは頂点が交点のTolerance範囲の近傍んあるとして頂点で代用）から伸びる方向を前方向としておく
							// Valueの方が交点から遠い方とする
							if ((CandidateEdge.Key - Positions[ParticleIdx]).SizeSquared() > (CandidateEdge.Value - Positions[ParticleIdx]).SizeSquared())
							{
								FVector SwapTmp = CandidateEdge.Value;
								CandidateEdge.Value = CandidateEdge.Key;
								CandidateEdge.Key = SwapTmp;
							}

							double DotProduct = FVector::DotProduct((CandidateEdge.Value - CandidateEdge.Key).GetSafeNormal(), -TensionDir);
							if (DotProduct > MaxDotProduct)
							{
								MaxDotProduct = DotProduct;
								MoveTargetEdgeIdx = MoveAlongEdgeIdx;
							}
						}

						if (MoveTargetEdgeIdx != INDEX_NONE)
						{
							//TODO: 本来は再帰的検索が必要だが一旦テストのため
							TArray<int32> OuterCornerEdges;
							for (int32 PairIdx = 0; PairIdx < EdgePairs.Num(); PairIdx++)
							{
								if (CornerTypes[PairIdx] == CornerType::OuterCorner
									&& ((EdgePairs[PairIdx].Key == MoveTargetEdgeIdx) || (EdgePairs[PairIdx].Value == MoveTargetEdgeIdx)))
								{
									// Crossしている順番にソート
									if (CornerEdgeIdxInfos[PairIdx] == EdgePairs[PairIdx].Key)
									{
										OuterCornerEdges.AddUnique(EdgePairs[PairIdx].Key);
										OuterCornerEdges.AddUnique(EdgePairs[PairIdx].Value);
									}
									else
									{
										OuterCornerEdges.AddUnique(EdgePairs[PairIdx].Value);
										OuterCornerEdges.AddUnique(EdgePairs[PairIdx].Key);
									}
								}
							}

							if (OuterCornerEdges.IsEmpty())
							{
								if (EdgeIdxOfPositions[ParticleIdx - 1] == MoveTargetEdgeIdx
									|| EdgeIdxOfPositions[ParticleIdx + 1] == MoveTargetEdgeIdx)
								{
									// エッジ移動先を前の頂点かあとの頂点が占めていれば削除
									PrevPositions.RemoveAt(ParticleIdx);
									Positions.RemoveAt(ParticleIdx);
									EdgeIdxOfPositions.RemoveAt(ParticleIdx);
									MovedFlagOfPositions.RemoveAt(ParticleIdx);

									// TODO:同じParticleIdxでループしたいからだが、変数名変えよう
									bParticleIdxLoopAgain = true;
								}
								else
								{
									// エッジ移動先に選ばれたエッジにOuterCornerがなければ、そのままエッジ移動
									EdgeIdxOfPositions[ParticleIdx] = MoveTargetEdgeIdx;
									MovedFlagOfPositions[ParticleIdx] = true;

									PrevPositions[ParticleIdx] = Positions[ParticleIdx];
								}
							}
							else
							{
								//TODO: もしConcaveになっているOuterCornerエッジを頂点追加したとしても、MovedFlagOfPositionsは
								//立てるので、次のイテレーションで頂点削除される想定
								//TODO: ここはConvex検出で頂点追加するアイディアもあるが、Convex検出するには先にMovementPhaseを
								//経て交差点から移動しておく必要があり、そうすると大きく動くと結構めりこみうるのでConvex検出でも
								//検出できない恐れはある

								// TODO:OuterCornerEdges内のソートが必要。現状の実装で正確に動作するのはOuterCornerがひとつのときだけ
								check(OuterCornerEdges.Num() >= 2);

								int32 PreParticleEdgeIdx = EdgeIdxOfPositions[ParticleIdx - 1];
								int32 PostParticleEdgeIdx = EdgeIdxOfPositions[ParticleIdx + 1];
								if (PreParticleEdgeIdx == OuterCornerEdges[0]
									|| PostParticleEdgeIdx == OuterCornerEdges[0]) //TODO:こんな分岐で処理するのも雑。仮実装。
								{
									// エッジ移動先を前の頂点かあとの頂点が占めていれば削除
									PrevPositions.RemoveAt(ParticleIdx);
									Positions.RemoveAt(ParticleIdx);
									EdgeIdxOfPositions.RemoveAt(ParticleIdx);
									MovedFlagOfPositions.RemoveAt(ParticleIdx);

									// TODO:同じParticleIdxでループしたいからだが、変数名変えよう
									bParticleIdxLoopAgain = true;
								}
								else
								{
									EdgeIdxOfPositions[ParticleIdx] = OuterCornerEdges[0];
									MovedFlagOfPositions[ParticleIdx] = true;

									PrevPositions[ParticleIdx] = Positions[ParticleIdx];
								}

								for (int32 i = 1; i < OuterCornerEdges.Num(); i++)
								{
									if (PreParticleEdgeIdx != OuterCornerEdges[i]
										&& PostParticleEdgeIdx != OuterCornerEdges[i]) //TODO:こんな分岐で処理するのも雑。仮実装。
									{
										PrevPositions.Insert(Positions[ParticleIdx], ParticleIdx + i);
										// PositionsにPositionsの要素をInsertするとTArrayの内部でチェックにひっかかるので
										Positions.Insert(FVector(Positions[ParticleIdx]), ParticleIdx + i); 
										EdgeIdxOfPositions.Insert(OuterCornerEdges[i], ParticleIdx + i);
										MovedFlagOfPositions.Insert(true, ParticleIdx + i); // ここでtrueにして次イテレーションで最短コンストレイントを行う // TODO:逆順ループを作れば次イテレーションにしなくてもよくなり収束も早くなるかも

										bParticleIdxLoopAgain = true;
									}
								}
							}
						}
						else // MoveTargetEdgeIdx == INDEX_NONE
						{
							PrevPositions[ParticleIdx] = Positions[ParticleIdx];
						}
						// TODO: MoveTargetEdgeIdx == INDEX_NONEは、すべてがSideEdgeで次に頂点が削除されるようなケースでありうる
						// TODO: 頂点削除は始点終点のセグメントは今のやり方でもいいが、本来はConvex検出でやるべき？コリジョン側が
						// 動くケースも考慮必要


						// TODO:移動先エッジにつらなるOuterCornerがエッジペアにあれば、それも移動先として採用。
					}
				}
				//
				// 始点終点以外のセグメントのCollisionPhase　ここまで
				// 
			}
			//
			// CollisionPhase　ここまで
			//

			else // bNeedCollisionPhaseがflaseのときの処理
			{
				check(!bParticleIdxLoopAgain);
				PrevPositions[ParticleIdx] = Positions[ParticleIdx];
			}
		}
		//
		// パーティクルループ　ここまで
		//

		//
		// イテレーション早期打ち切りのための動いた頂点があるかどうかの判定
		//
		bExistMovedParticle = false;
		for (int32 ParticleIdx = 0; ParticleIdx < Positions.Num() - 1; ParticleIdx++)
		{
			if (MovedFlagOfPositions[ParticleIdx])
			{
				bExistMovedParticle = true;
				break;
			}
		}
	}
	//
	// イテレーションループ　ここまで
	//
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

