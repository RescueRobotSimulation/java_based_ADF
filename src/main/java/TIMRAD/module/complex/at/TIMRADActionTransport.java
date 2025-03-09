
import adf.core.agent.action.Action;
import adf.core.agent.action.ambulance.ActionLoad;
import adf.core.agent.action.ambulance.ActionUnload;
import adf.core.agent.action.common.ActionMove;
import adf.core.agent.action.common.ActionRest;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.information.MessageFireBrigade;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.communication.CommunicationMessage;
import adf.core.component.extaction.ExtAction;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import rescuecore2.config.NoSuchConfigOptionException;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import static rescuecore2.standard.entities.StandardEntityURN.*;

public class TIMRADActionTransport extends ExtAction {
    private final PathPlanning pathPlanning;
    private final int thresholdRest;
    private final int kernelTime;
    private EntityID target;
    private final ExtAction actionExtMove;
    private final MessageManager messageManager;
    private int unloadTime = 0;
    

    @Override
   public TIMRADActionTransport(AgentInfo agentInfo, WorldInfo worldInfo, ScenarioInfo scenarioInfo,
                                    ModuleManager moduleManager, DevelopData developData) {
        super(agentInfo, worldInfo, scenarioInfo, moduleManager, developData);
        
        this.thresholdRest = developData.getInteger("ActionTransport.rest", 100);
        this.pathPlanning = moduleManager.getModule("DefaultExtActionTransport.PathPlanning", "adf.impl.module.algorithm.AStarPathPlanning");
        this.actionExtMove = moduleManager.getExtAction("DefaultTactics.ExtActionMove", "adf.impl.extaction.OptimizedExtActionMove");
        this.messageManager = new MessageManager();
        
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        } catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
    }



     public ExtAction precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        if (this.getCountPrecompute() >= 2) {
            return this;
        }
        this.pathPlanning.precompute(precomputeData);
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        } catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
        return this;
    }
    public ExtAction resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if (this.getCountResume() >= 2) {
            return this;
        }
        this.pathPlanning.resume(precomputeData);
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        } catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
        return this;
    }

    public ExtAction preparate() {
        super.preparate();
        if (this.getCountPreparate() >= 2) {
            return this;
        }
        this.pathPlanning.preparate();
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        } catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
        return this;
    }

    public ExtAction updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);
        if (this.messageManager==null){
            this.messageManager = messageManager;
        }
        if (this.getCountUpdateInfo() >= 2) {
            return this;
        }
        this.pathPlanning.updateInfo(messageManager);
        return this;
    }

    @Override
    public ExtAction calc() {
        this.result = null;
        this.me = (AmbulanceTeam) this.agentInfo.me();
        this.transportHuman = this.agentInfo.someoneOnBoard();

        if (transportHuman != null) {
            this.result = this.calcUnload();
        } else if (this.needRest(me)) {
            this.result = this.calcRefugeAction();
        } else if (this.target != null) {
            this.result = this.calcRescue();
        }
        
        return this;
    }

    private Action calcUnload() {
        if (this.target == null) {
            return null;
        }
        
        if (this.target.equals(this.me.getPosition())) {
            return new ActionUnload();
        }
        
        List<EntityID> path = this.pathPlanning.getPath(this.me.getPosition(), this.target);
        if (path != null && !path.isEmpty()) {
            return new ActionMove(path);
        }
        
        return null;
    }

    private Action calcRefugeAction() {
        EntityID refuge = this.findNearestRefuge();
        if (refuge != null) {
            List<EntityID> path = this.pathPlanning.getPath(this.me.getPosition(), refuge);
            if (path != null && !path.isEmpty()) {
                return new ActionMove(path);
            }
        }
        return null;
    }

    private Action calcRescue() {
        if (this.target == null) {
            return null;
        }
        
        if (this.target.equals(this.me.getPosition())) {
            return new ActionLoad(target);
        }
        
        List<EntityID> path = this.pathPlanning.getPath(this.me.getPosition(), this.target);
        if (path != null && !path.isEmpty()) {
            return new ActionMove(path);
        }
        
        return null;
    }

    private EntityID findNearestRefuge() {
        Collection<StandardEntity> refuges = this.worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE);
        EntityID nearest = null;
        double minDistance = Double.MAX_VALUE;

        for (StandardEntity refuge : refuges) {
            double distance = this.worldInfo.getDistance(this.me.getPosition(), refuge.getID());
            if (distance < minDistance) {
                minDistance = distance;
                nearest = refuge.getID();
            }
        }
        
        return nearest;
    }
}

