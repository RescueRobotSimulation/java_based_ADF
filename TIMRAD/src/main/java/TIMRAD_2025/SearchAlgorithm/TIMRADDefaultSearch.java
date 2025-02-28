package TIMRAD_2025.SearchAlgorithm;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;

import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

/**
  * @author Ikegami(18th)
  * @since  2023.3.20
  * @brief  Searchの目的地を求める
  */
public class TIMRADDefaultSearch {
  protected ScenarioInfo scenarioInfo;
  protected WorldInfo    worldInfo;
  protected AgentInfo    agentInfo;
  protected PathPlanning pathPlanning;
  protected Clustering   clustering;
  protected MessageManager messageManager;
  private   EntityID target;
  
  /**
   * @brief  このインスタンスを利用するエージェントの情報を引き受ける
   * @return IRIODefaultSearch this を想定
   */
  public TIMRADDefaultSearch setInfo(ScenarioInfo scenarioInfo, WorldInfo worldInfo, AgentInfo agentInfo) {
    this.scenarioInfo = scenarioInfo;
    this.worldInfo    = worldInfo;
    this.agentInfo    = agentInfo;
    return this;
  }

  /**
   * @brief  このインスタンスを利用するエージェントが使うモジュールを引き受ける
   * @return IRIODefaultSearch this を想定
   */
  public TIMRADDefaultSearch setAlgorithm(PathPlanning pathPlanning, Clustering clustering) {
    this.pathPlanning = pathPlanning;
    this.clustering   = clustering;
    return this;
  }

  /**
   * @return IRIODefaultSearch this を想定
   * @brief  各サイクルごとのアップデート処理
   * @note   エージェントのリスト取得など, 対象に関係な胃共通の処理はここで実装しよう
   */
  public TIMRADDefaultSearch updateInfo(MessageManager messageManager) {
    this.messageManager = messageManager;
    return this;
  }

  /**
   * @return RIODefaultSearch
   * @brief  評価対象に対して優先度・不安定度を求める
   */
  public TIMRADDefaultSearch calc() {
    return this;
  }

  public EntityID getTarget() {
    return target;
  }
}
