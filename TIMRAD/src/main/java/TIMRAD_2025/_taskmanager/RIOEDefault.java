package TIMRAD_2025._taskmanager;

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


public abstract class RIOEDefault implements IRIOEvaluator {
    protected boolean hasEvaluated = false;

    protected ScenarioInfo scenarioInfo;
    protected WorldInfo worldInfo;
    protected AgentInfo agentInfo;
    protected PathPlanning pathPlanning;
    protected Clustering clustering;
    protected MessageManager messageManager;

    protected EntityID evaluateeID;

    
    @Override
    public final IRIOEvaluator setInfo(ScenarioInfo scenarioInfo, WorldInfo worldInfo, AgentInfo agentInfo) {
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

    protected IRIOEvaluator init() {
        return this;
    }

    
    @Override
    public IRIOEvaluator update() {
        this.hasEvaluated = false;
        return this;
    }

    /**
     */
    @Override
    public final RIODynamicTaskKey calcNormalized(EntityID evaluateeID) {
        RIODynamicTaskKey result = this.calc(evaluateeID);
        double priority = normalize(result.priority, this.bestPriority(), this.worstPriority());
        double unstableness = normalize(result.unstableness, this.bestUnstableness(), this.worstUnstableness());
        return new RIODynamicTaskKey(evaluateeID, priority, unstableness);
    }

    /**
     */
    @Override
    public abstract RIODynamicTaskKey calc(EntityID evaluateeID);

    protected abstract double bestPriority();

    protected abstract double worstPriority();

    protected abstract double bestUnstableness();

    protected abstract double worstUnstableness();

    /**
     * @brief 
     */
    private double normalize(double value, double best, double worst) {
        return (value - best) / (worst - best);
    }

    @Override
    public final IRIOEvaluator setMessageManager(MessageManager messageManager){
        this.messageManager = messageManager;
        return this;
    };
}
