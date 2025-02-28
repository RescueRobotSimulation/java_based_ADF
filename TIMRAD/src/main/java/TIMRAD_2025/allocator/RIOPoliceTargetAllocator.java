package TIMRAD_2025.allocator;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.module.complex.PoliceTargetAllocator;
import rescuecore2.standard.entities.FireBrigade;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class RIOPoliceTargetAllocator extends PoliceTargetAllocator {
    public RIOPoliceTargetAllocator(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
    }

    @Override
    public PoliceTargetAllocator resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if(this.getCountResume() >= 2) {
            return this;
        }
        return this;
    }

    @Override
    public PoliceTargetAllocator preparate() {
        super.preparate();
        if(this.getCountPrecompute() >= 2) {
            return this;
        }
        return this;
    }

    @Override
    public Map<EntityID, EntityID> getResult() {
        return new HashMap<>();
    }

    @Override
    public PoliceTargetAllocator calc() {
        return this;
    }

    @Override
    public PoliceTargetAllocator updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);
        if(this.getCountUpdateInfo() >= 2) {
            return this;
        }
        return this;
    }

    private List<StandardEntity> getActionAgents(Map<EntityID, PoliceForceInfo> infoMap) {
        List<StandardEntity> result = new ArrayList<>();
        int agentsNumber = this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE).size();
        double useNumber = agentsNumber*0.8;//全体の八割のエージェントに命令する
        int count = 0;
        for (StandardEntity entity : this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE)) {
            if(count>useNumber){
                break;//8割に越えたら終わる
            }
            PoliceForceInfo info = infoMap.get(entity.getID());
            if (info != null && info.canNewAction && ((FireBrigade) entity).isPositionDefined() ) {
                result.add(entity);
                count++;
            }
        }
        return result;
    }

    private class PoliceForceInfo {
        EntityID agentID;
        EntityID target;
        boolean canNewAction;
        int commandTime;

        PoliceForceInfo(EntityID id) {
            agentID = id;
            target = null;
            canNewAction = true;
            commandTime = -1;
        }
    }
}
