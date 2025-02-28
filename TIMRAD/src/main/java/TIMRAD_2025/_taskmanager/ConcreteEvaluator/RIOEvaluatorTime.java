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
 * @since 2020
 * 不安定度を時間に伴って増加させる
 */
public class RIOEvaluatorTime extends RIOEDefault {
    private Map<EntityID, Double> evaluateeMap = new HashMap<EntityID, Double>();

    /// ---- 正規化関数 ----
    /// summary : 最も良い値 = 1（とらないけど）
    @Override
    protected double bestPriority() {
        return 1;
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

    /// summary : 最も安定 = 1
    @Override
    protected double worstUnstableness() {
        return 0;
    }

    /// ---- 各種計算 ----
    /// summary : 各サイクルごとの事前処理
    @Override
    public IRIOEvaluator update(){
        // サイクルごとに登録時間の長さを+1する
        for (Map.Entry<EntityID, Double> entry : evaluateeMap.entrySet()){
            this.evaluateeMap.put(entry.getKey(), entry.getValue().doubleValue() + 1);
        }
        return this;
    }

    /// summary : 優先度(ココでは利用しないので0を返す), 不安定度計算(時間の逆数として考える)
    @Override
    public RIODynamicTaskKey calc(EntityID evaluateeID) {
        double priority = 0;
        double unstableness;
        if(this.evaluateeMap.containsKey(evaluateeID)){
            unstableness = 1.0 / this.evaluateeMap.get(evaluateeID).doubleValue();
        } else {
            //未登録の場合
            this.evaluateeMap.put(evaluateeID, 1.0);
            unstableness = 1.0;
        }

        return new RIODynamicTaskKey(null, priority, unstableness);
    }

    /// ---- メンバメソッド ----
    /// summary :
}
