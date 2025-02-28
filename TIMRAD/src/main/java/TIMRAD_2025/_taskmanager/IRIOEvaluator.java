/**
  * @author Horie(16th)
  * @since 2020.10.-
  * summary : 動的なtarget選択のために, Comparatorと優先度管理クラスを実装
  * 優先度と, 優先度の変化しやすさ（＝不安定度）を各対象ごとに求め管理する
  *
  */

package TIMRAD_2025._taskmanager;

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
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.complex.RoadDetector;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

/**
  * @author Horie(16th)
  * @since  2020.10.-
  * @brief  各基準ごとに, 優先度と優先度の変わりやすさ（不安定度）を算出する
  *         interfaceであるので, これを継承して評価値計算クラスを作る
  * @note   実際に使うときは, 具象クラスではなく,
  *         このインターフェースのインスタンスとして各インスタンスを定義する
  */

public interface IRIOEvaluator{


    /**
     * @brief  このインスタンスを利用するエージェントの情報を引き受ける
     * @return IRIOEvaluator this を想定
     */
    public IRIOEvaluator setInfo(ScenarioInfo scenarioInfo, WorldInfo worldInfo, AgentInfo agentInfo);

    /**
     * @brief  このインスタンスを利用するエージェントが使うモジュールを引き受ける
     * @return IRIOEvaluator this を想定
     */
    public IRIOEvaluator setAlgorithm(PathPlanning pathPlanning, Clustering clustering);

    /**
    * @brief このインスタンスを利用するエージェントのmessageManagerを引き受ける
     */
    public IRIOEvaluator setMessageManager(MessageManager messageManager);

    /**
     * @return IRIOEvaluator this を想定
     * @brief  各サイクルごとのアップデート処理
     * @note   エージェントのリスト取得など, 対象に関係な胃共通の処理はここで実装しよう
     */
    public IRIOEvaluator update();

    /**
     * @return RIODynamicTaskKey
     * @brief  評価対象に対して優先度・不安定度を求める
     * @note   return は new RIODynamicTaskkey(id, 優先度, 不安定度)
     */
    public RIODynamicTaskKey calc(EntityID evaluateeID);

    /**
     * @return RIODynamicTaskKey
     * @brief  評価対象に対して「正規化した」優先度・不安定度を求める
     * @note   RIOEDefaultで実装しているので, 基本的にみんなが実装する必要はない
     */
    public RIODynamicTaskKey calcNormalized(EntityID evaluateeID);



}