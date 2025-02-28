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
 * @author Horie(16th)
 * @since 2020 建物の入り口にある瓦礫をさがし, 種類と人の数で評価 2019 以前の RoadDetector
 *        実装部分を動的タスクシステム管理に移植したもの
 */
public class RIOEvaluatorBuildingEntrance extends RIOEDefault {
    private final double weightRefuge = 1000; //// attention : マジックナンバー
    private List<EntityID> buildings;

    /// ---- 正規化関数 ----
    /// summary : 最も良い値 = refugeだったときの重み * 見渡せるすべての人間
    @Override
    protected double bestPriority() {
        return this.weightRefuge * this.worldInfo.getEntityIDsOfType(StandardEntityURN.BUILDING).size();
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
        this.buildings = new ArrayList<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.BUILDING)); // 全ての「建物」
        return this;
    }

    /// summary : 優先度計算(編集中), 不安定度計算(ココでは何もしない)
    @Override
    public RIODynamicTaskKey calc(EntityID evaluateeID) {
        double priority = this.getEntrancePriority(evaluateeID);
        double unstableness = 0;
        return new RIODynamicTaskKey(null, priority, unstableness);
    }

    /// ---- メンバメソッド ----
    /// summary : 与えられた候補に対して優先度を計算する
    private double getEntrancePriority(EntityID evaluateeID) {
        // 建物に隣接するエリアがRoadで, かつ瓦礫が存在する場合のみ考慮
        StandardEntity entranceEntity = this.worldInfo.getEntity(evaluateeID);
        if (!(entranceEntity instanceof Road)) {
            return 0;
        }
        Road entrance = (Road) entranceEntity;
        if (!entrance.isBlockadesDefined() || entrance.getBlockades().size() <= 0) {
            return 0;
        }

        /// 本処理
        double priority = 0;
        for (EntityID buildingID : this.buildings) {
            // 1.buildingIDの型チェック
            // buildingをArea型で取得
            StandardEntity building = worldInfo.getEntity(buildingID);
            if (!(building instanceof Area)) {
                continue;
            }
            Area area = (Area) building;

            // 建物にevaluateeIDが隣接する場合のみ考慮
            List<EntityID> neighbours = area.getNeighbours();
            if (neighbours == null || !neighbours.contains(evaluateeID)) {
                continue;
            }

            // 2.areaがどんな建物なのかチェック
            // 建物の種類や埋まっている人の数で評価
            if (building instanceof Refuge) {
                priority += this.weightRefuge * this.worldInfo.getNumberOfBuried(evaluateeID);
            } else {
                priority += this.worldInfo.getNumberOfBuried(evaluateeID);
            }
        }
        return priority;
    }
}
