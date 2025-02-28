package TIMRAD_2025._taskmanager.sandbox;

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
 * @author Horie(16th)
 * @since 2020 2019 以前の RoadDetector 実装部分を動的タスクシステム管理に移植したもの
 */
public class RIOEAroundCrowdAns extends RIOEDefault {
    private final double weightCrowd = 1000; // attention : マジックナンバー
    private double clearDistance;

    // ArrayListではなく削除・検索が高速な(Hash)Setを使う
    private Set<EntityID> civilians; // 全ての「市民」
    private Set<EntityID> AmbulanceTeams; // 全ての「AT」
    private Set<EntityID> fireBrigades; // 全ての「FB」

    @Override
    protected IRIOEvaluator init(){
        this.clearDistance = this.scenarioInfo.getClearRepairDistance();
        return this;
    }

    /// ---- 正規化関数 ----
    /// summary : 最も良い値 = 重み * (エージェントの人数^2 * 市民の人数)
    @Override
    protected double bestPriority() {
        return weightCrowd * (this.worldInfo.getEntityIDsOfType(StandardEntityURN.CIVILIAN).size()
                + Math.pow(this.worldInfo.getEntityIDsOfType(StandardEntityURN.FIRE_BRIGADE).size()
                + this.worldInfo.getEntityIDsOfType(StandardEntityURN.AMBULANCE_TEAM).size(), 2));
    }

    /// summary : 最も悪い値 = 0
    @Override
    protected double worstPriority() {
        return 0;
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
    /// summary : 各サイクルごとの事前処理
    @Override
    public IRIOEvaluator update(){
        this.civilians = new HashSet<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.CIVILIAN)); // 全ての「市民」
        this.AmbulanceTeams = new HashSet<>(
                    this.worldInfo.getEntityIDsOfType(StandardEntityURN.AMBULANCE_TEAM)); // 全ての「AT」
        this.fireBrigades = new HashSet<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.FIRE_BRIGADE)); // 全ての「FB」
        return this;
    }

    /// summary : 優先度計算(編集中), 不安定度計算(ココでは何もしない)
    @Override
    public RIODynamicTaskKey calc(EntityID evaluateeID) {
        double priority = this.getCrowd(evaluateeID);
        double unstableness = 0;
        return new RIODynamicTaskKey(null, priority, unstableness);
    }

    private double getCrowd(EntityID evaluateeID) {
        // 建物に隣接するエリアがRoadで, かつ瓦礫が存在する場合のみ考慮
        StandardEntity evaluateeEntity = this.worldInfo.getEntity(evaluateeID);
        if (!(evaluateeEntity instanceof Road)) {
            return 0;
        }
        Road evaluatee = (Road) evaluateeEntity;
        if (!evaluatee.isBlockadesDefined() || evaluatee.getBlockades().size() <= 0) {
            return 0;
        }

        // 本処理
        int humanWeight = 0;
        int agentWeight = 0; // 「瓦礫付近のFBとAT」の数
        int border = this.worldInfo.getChanged().getChangedEntities().size(); // 基準点（視界内の「人」の数）
        double priority = 0;

        for (EntityID civilian : this.civilians) {
            if (this.worldInfo.getDistance(evaluateeID, civilian) <= this.clearDistance) { // 「瓦礫」付近の「市民」を数える
                humanWeight += 1;
            }
        }
        for (EntityID ambulanceTeam : this.AmbulanceTeams) {
            if (this.worldInfo.getDistance(evaluateeID, ambulanceTeam) <= this.clearDistance) { // 「瓦礫」付近の「AT」を数える
                agentWeight += 1;
            }
            // if (worldInfo.getDistance(ambulanceTeam, civilian) == 0) ; //
            // 背負っているやつも別でカウントすべき？
        }

        for (EntityID fireBrigade : this.fireBrigades) {
            if (this.worldInfo.getDistance(evaluateeID, fireBrigade) <= this.clearDistance) { // 「瓦礫」付近の「FB」を数える
                agentWeight += 1;
            }
        }

        priority = humanWeight + Math.pow(agentWeight, 2);

        // 「瓦礫」付近に「人」が大勢いる（全ての「視界内の人」の半数以上が「瓦礫」付近にいる）場合はより優先
        if (humanWeight >= border / 2 + 1) {
            return priority * this. weightCrowd;
        }
        return priority;

        /*
         * EntityID target = null; if (!targets.isEmpty()) // 「大勢のの人がいる道を塞いでいる瓦礫」が存在する場合
         * { target = this.getPathToNearestTarget(positionID, targets); }
         */
    }

}
