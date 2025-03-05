package TIMRAD.module.complex.search;

import adf.core.agent.action.common.ActionMove;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import adf.core.component.module.complex.Search;
import com.mrl.debugger.remote.VDClient;
import TIMRAD.TIMRADConstants;
import TIMRAD.viewer.TIMRADPersonalData;
import TIMRAD.world.TIMRADWorldHelper;
import TIMRAD.world.entity.Path;
import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;

import java.io.Serializable;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import static rescuecore2.standard.entities.StandardEntityURN.*;

/**
 * @author Shiva
 */

public class TIMRADSearchManager extends Search {
    private static final Log logger = LogFactory.getLog(TIMRADSearchManager.class);
    private static final int EXECUTE_LIMIT = 10;
    private final TIMRADWorldHelper world;
    private final TIMRADnSearchDecisionMaker timradSearchDecisionMaker;
    private final TIMRADSimpleSearchDecisionMaker timradSimpleSearchDecisionMaker;
    private final TIMRADSearchStrategy timradStrategy;
    private final PathPlanning pathPlanning;
    private final Clustering clustering;

    private EntityID target;
    private Area targetArea;
    private Path targetPath;
    private int lastUpdateTime;
    private int lastExecuteTime;
    private int thisCycleExecute;
    private int simpleDMUpdateTime;
    private boolean needChange;

    public TIMRADSearchManager(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
        this.world = TIMRADWorldHelper.load(ai, wi, si, moduleManager, developData);

        this.pathPlanning = moduleManager.getModule("SampleSearch.PathPlanning", "adf.impl.module.algorithm.AStarPathPlanning");
        this.clustering = moduleManager.getModule("SampleSearch.Clustering", "adf.impl.module.algorithm.KMeansClustering");

        this.timradSearchDecisionMaker = new TIMRADSearchDecisionMaker(world, ai, wi);
        this.timradSimpleSearchDecisionMaker = new TIMRADSimpleSearchDecisionMaker(world, ai, wi);
        this.timradStrategy = new TIMRADSearchStrategy(world, pathPlanning, wi, ai);
    }

    @Override
    public Search updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);
        this.world.updateInfo(messageManager);
        this.pathPlanning.updateInfo(messageManager);
        this.clustering.updateInfo(messageManager);
        this.timradSearchDecisionMaker.update();
        return this;
    }

    @Override
    public Search calc() {
        if (agentInfo.getTime() < scenarioInfo.getKernelAgentsIgnoreuntil()) {
            return this;
        }
        execute();
        if (target == null) {
            target = getSimpleSearchTarget();
        }
        return this;
    }

    private void execute() {
        if (world.getTime() == lastExecuteTime) {
            thisCycleExecute++;
        } else {
            lastExecuteTime = world.getTime();
            thisCycleExecute = 0;
        }
        if (thisCycleExecute > EXECUTE_LIMIT) {
            return;
        }

        if (isNeedToChangeTarget()) {
            if (isNeedUpdateDecisionMaker()) {
                lastUpdateTime = world.getTime();
                timradSearchDecisionMaker.update();
            }
            needChange = false;
            targetArea = timradSearchDecisionMaker.getNextArea();
            target = (targetArea != null) ? targetArea.getID() : null;
        } else {
            Area betterTarget = sivilianSearchDecisionMaker.getBetterTarget(targetArea);
            if (betterTarget != null) {
                targetArea = betterTarget;
                target = targetArea.getID();
            }
        }
        if (targetArea != null) {
            searchStrategy.setSearchingPath(targetPath, true);
            ActionMove action = (ActionMove) searchStrategy.searchBuilding((Building) targetArea);
            if (action != null && !action.getUsePosition()) {
                target = (!action.getPath().isEmpty() ? action.getPath().get(action.getPath().size() - 1) : null);
            }
        }
        if (target == null) {
            needChange = true;
            execute();
        }
    }

    private EntityID getSimpleSearchTarget() {
        if (simpleDMUpdateTime < agentInfo.getTime()) {
            simpleDMUpdateTime = agentInfo.getTime();
            timradSimpleSearchDecisionMaker.update();
        }
        if (targetPath == null) {
            targetPath = timradSimpleSearchDecisionMaker.getNextPath();
        }
        searchStrategy.setSearchingPath(targetPath, true);
        ActionMove action = (ActionMove) searchStrategy.searchPath();
        if (action != null && !action.getUsePosition()) {
            return (action.getPath() != null && !action.getPath().isEmpty() ? action.getPath().get(action.getPath().size() - 1) : null);
        }
        targetPath = null;
        return null;
    }

    private boolean isNeedToChangeTarget() {
        return targetArea == null || world.getMrlBuilding(targetArea.getID()).isVisited() || needChange;
    }

    private boolean isNeedUpdateDecisionMaker() {
        return lastUpdateTime < world.getTime() || needChange;
    }
}

