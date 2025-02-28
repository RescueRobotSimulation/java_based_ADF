package TIMRAD_2025._taskmanager.ConcreteEvaluator;

import adf.core.agent.Agent;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.MessageUtil;
import adf.core.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.core.agent.communication.standard.bundle.information.MessageAmbulanceTeam;
import adf.core.agent.communication.standard.bundle.information.MessageFireBrigade;
import adf.core.agent.communication.standard.bundle.information.MessagePoliceForce;
import adf.core.agent.communication.standard.bundle.information.MessageRoad;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.communication.CommunicationMessage;
import adf.core.component.module.algorithm.PathPlanning;
import adf.core.component.module.complex.RoadDetector;
//import jdk.javadoc.internal.doclets.toolkit.resources.doclets;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

import TIMRAD_2025._taskmanager.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

/**
 * @author Mohsen
 * @since 2024  RoadDetector 
 */
public class RIOEvaluatorAroundFire extends RIOEDefault {
    private final double weightFar = 1000; //// attention : マジックナンバー
    private double longestDistance;
    private double extinguishDistance;
    private List<EntityID> buildings; // 全ての「建物」

    @Override
    protected IRIOEvaluator init(){
        longestDistance = this.scenarioInfo.getPerceptionLosMaxDistance();
        extinguishDistance = this.scenarioInfo.getFireExtinguishMaxDistance();
        return this;
    }

    /// ---- 正規化関数 ----
    /// summary : 最も良い値 = 0
    @Override
    protected double bestPriority() {
        return 0;
    }

    /// summary : 最も悪い値 = 最も遠いキョリ * 建物の数 * 優先しない場合の重み
    @Override
    protected double worstPriority() {
        return this.longestDistance * this.weightFar
                * this.worldInfo.getEntityIDsOfType(StandardEntityURN.BUILDING).size();
    }

    /// summary : 最も不安定 = 0
    @Override
    protected double bestUnstableness() {
        return 0;
    }

    /// summary : 最も安定 = 1（使わないけど）
    @Override
    protected double worstUnstableness() {
        return 0;
    }

    /// ---- 各種計算 ----
    /// summary : 各サイクルごとの事前処理, buildingsを更新
    @Override
    public IRIOEvaluator update(){
        this.buildings = new ArrayList<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.BUILDING));
        return this;
    }

    /// summary : 優先度計算(編集中), 不安定度計算(ココでは何もしない)
    @Override
    public RIODynamicTaskKey calc(EntityID evaluateeID) {
        double priority = this.getAroundFire(evaluateeID);
        double unstableness = 0;
        return new RIODynamicTaskKey(null, priority, unstableness);
    }


    /// ---- メンバメソッド ----
    private double getAroundFire(EntityID evaluateeID) {
        // 建物に隣接するエリアがRoadで, かつ瓦礫が存在する場合のみ考慮
        StandardEntity evaluateeEntity = this.worldInfo.getEntity(evaluateeID);
        if (!(evaluateeEntity instanceof Road)) {
            return this.worstPriority();
        }
        Road evaluatee = (Road) evaluateeEntity;
        if (!evaluatee.isBlockadesDefined() || evaluatee.getBlockades().size() <= 0) {
            return this.worstPriority();
        }

        /// 本処理
        double priority = 0;
        for (EntityID buildingEntity : this.buildings) {
            Building building = (Building) this.worldInfo.getEntity(buildingEntity);

            /// 燃えてない建物の場合は最悪値を設定
            if (building == null || !building.isOnFire() || !building.isFierynessDefined()
                    || building.getFieryness() <= 0) {
                priority += this.longestDistance * this.weightFar;
                continue;
            }

            // キョリ : 「道」が「瓦礫」で塞がれており、「建物」から近い場合
            double distance = this.worldInfo.getDistance(buildingEntity, evaluateeID);

            // 消火可能キョリより遠い瓦礫に対しては優先度を低くする（値を大きくする）
            if (this.extinguishDistance < distance) {
                priority += distance * this.weightFar;
            } else {
                priority += distance;
            }
        }

        /*
         * EntityID target = null;
         *
         * if (!targets.isEmpty()) // 「燃えている建物の周囲の瓦礫で塞がれている道」が存在する場合 { target =
         * this.getPathToNearestTarget(positionID, targets); if (target != null) {
         * return target; } }
         */

        return priority;
    }
}
