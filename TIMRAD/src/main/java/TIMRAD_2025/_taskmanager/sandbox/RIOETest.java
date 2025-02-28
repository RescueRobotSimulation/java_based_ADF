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
 * @since 2020
 * @brief Evaluatorの動作をチェックするためのクラス
 */
public class RIOETest extends RIOEDefault {
    private final double weightFar = 1000; //// attention : マジックナンバー
    private double longestDistance;
    private double extinguishDistance;
    private List<EntityID> buildings; // 全ての「建物」

    // 最初の最初だけに呼び出し
    @Override
    protected IRIOEvaluator init(){
        System.out.println("init() ... 一番最初のみ");
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
        return 1;
    }

    /// summary : 最も不安定 = 0
    @Override
    protected double bestUnstableness() {
        return 0;
    }

    /// summary : 最も安定 = 1（使わないけど）
    @Override
    protected double worstUnstableness() {
        return 1;
    }

    /// ---- 各種計算 ----
    /// summary : 各サイクルごとの事前処理, buildingsを更新
    @Override
    public IRIOEvaluator update(){
        System.out.println("update() ... 各サイクルの最初のみ");
        return this;
    }

    /// summary : 優先度計算(編集中), 不安定度計算(ココでは何もしない)
    @Override
    public RIODynamicTaskKey calc(EntityID evaluateeID) {
        System.out.println("calc(EntityID) ... 各サイクルで候補一つずつに対して行われる");
        return new RIODynamicTaskKey(null, 0, 0);
    }
}
