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




public class RIOEvaluatorAroundCrowd extends RIOEDefault {
    private final double weightCrowd = 1000; // attention : 
    private double clearDistance;

    // (
    private Set<EntityID> civilians; // 
    private Set<EntityID> AmbulanceTeams; // 」
    private Set<EntityID> fireBrigades; // 

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

    /// summary :  
    @Override
    protected double worstPriority() {
        return 0;
    }

    /// summary :  
    @Override
    protected double bestUnstableness() {
        return 0;
    }

    /// summary : 
    @Override
    protected double worstUnstableness() {
        return 0;
    }

    /// ----  ----
    @Override
    public IRIOEvaluator update(){
        this.civilians = new HashSet<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.CIVILIAN)); // 全ての「市民」
        this.AmbulanceTeams = new HashSet<>(
                    this.worldInfo.getEntityIDsOfType(StandardEntityURN.AMBULANCE_TEAM)); // 全ての「AT」
        this.fireBrigades = new HashSet<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.FIRE_BRIGADE)); // 全ての「FB」
        return this;
    }

    @Override
    public RIODynamicTaskKey calc(EntityID evaluateeID) {
        double priority = this.getCrowd(evaluateeID);
        double unstableness = 0;
        return new RIODynamicTaskKey(null, priority, unstableness);
    }

    private double getCrowd(EntityID evaluateeID) {
        StandardEntity evaluateeEntity = this.worldInfo.getEntity(evaluateeID);
        if (!(evaluateeEntity instanceof Road)) {
            return 0;
        }
        Road evaluatee = (Road) evaluateeEntity;
        if (!evaluatee.isBlockadesDefined() || evaluatee.getBlockades().size() <= 0) {
            return 0;
        }

        // 
        int humanWeight = 0;
        int agentWeight = 0; // 
        int border = this.worldInfo.getChanged().getChangedEntities().size(); // 
        double priority = 0;

        for (EntityID civilian : this.civilians) {
            if (this.worldInfo.getDistance(evaluateeID, civilian) <= this.clearDistance) { // 
                humanWeight += 1;
            }
        }
        for (EntityID ambulanceTeam : this.AmbulanceTeams) {
            if (this.worldInfo.getDistance(evaluateeID, ambulanceTeam) <= this.clearDistance) { // 
                agentWeight += 1;
            }
        }

        for (EntityID fireBrigade : this.fireBrigades) {
            if (this.worldInfo.getDistance(evaluateeID, fireBrigade) <= this.clearDistance) { // 
                agentWeight += 1;
            }
        }

        priority = humanWeight + Math.pow(agentWeight, 2);

        // 
        if (humanWeight >= border / 2 + 1) {
            return priority * this. weightCrowd;
        }
        return priority;

        /*
         * EntityID target = null; if (!targets.isEmpty()) // 
         * { target = this.getPathToNearestTarget(positionID, targets); }
         */
    }

}
