//package RIO2023._taskmanager;

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
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.complex.RoadDetector;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

import TIMRAD_2025._taskmanager.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

/**
 * @deprecated
 * @author Horie(16th)
 * @since 2020.10.- summary : 
 * @Deprecated
 *
 *    
 */
public abstract class RIOEvaluatorNormalObsoleted implements IRIOEvaluator {
    // 評価のための情報
    private boolean hasEvaluated;
    private RIODynamicTaskKey evaluateResult;

    protected ScenarioInfo scenarioInfo;
    protected WorldInfo worldInfo;
    protected AgentInfo agentInfo;
    protected PathPlanning pathPlanning;
    protected Clustering clustering;
    protected MessageManager messageManager;

    // 
    protected EntityID evaluateeID;

   
    public final IRIOEvaluator setEvaluatee(EntityID evaluateeID) {
        this.evaluateeID = evaluateeID;
        this.evaluateResult = this.calc();
        return this;
    }

    @Override
    public final IRIOEvaluator setInfo(ScenarioInfo scenarioInfo, WorldInfo worldInfo, AgentInfo agentInfo) {
        // 
        this.scenarioInfo = scenarioInfo;
        this.worldInfo = worldInfo;
        this.agentInfo = agentInfo;
        this.init();
        return this;
    }

    @Override
    public final IRIOEvaluator setAlgorithm(PathPlanning pathPlanning, Clustering clustering) {
        this.pathPlanning = pathPlanning;
        this.clustering = clustering;
        return this;
    }

    /// summary 
    protected IRIOEvaluator init() {
        return this;
    }

    /// ----  ----
    /// summary : 
    @Override
    public IRIOEvaluator update() {
        this.hasEvaluated = false;
        return this;
    }

    /// summary : , 
    /// 
    public final double getPriority() {
        return (this.evaluateResult.priority - bestPriority()) / (worstPriority() - bestPriority());
    }

    /// sumamry : , 
    /// 
    public final double getUnstableness() {
        return (this.evaluateResult.unstableness - bestUnstableness()) / (worstUnstableness() - bestUnstableness());
    }

    /**
     */
    @Override
    public RIODynamicTaskKey calcNormalized(EntityID evaluateeID) {
        this.setEvaluatee(evaluateeID);
        return this.calc();
    }

    protected abstract RIODynamicTaskKey calc();

    /// summary : , 
    protected abstract double bestPriority();

    protected abstract double worstPriority();

    protected abstract double bestUnstableness();

    protected abstract double worstUnstableness();
}
