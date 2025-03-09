

public class TIMRADActionTransport extends ExtAction {
    private final PathPlanning pathPlanning;
    private final int thresholdRest;
    private final int kernelTime;
    private EntityID target;
    private final ExtAction actionExtMove;
    private final MessageManager messageManager;
    private int unloadTime = 0;
    public ActionTransport(PathPlanning pathPlanning) {
        this.pathPlanning = pathPlanning;
    }

    @Override
   public OptimizedActionTransport(AgentInfo agentInfo, WorldInfo worldInfo, ScenarioInfo scenarioInfo,
                                    ModuleManager moduleManager, DevelopData developData) {
        super(agentInfo, worldInfo, scenarioInfo, moduleManager, developData);
        
        this.thresholdRest = developData.getInteger("ActionTransport.rest", 100);
        this.pathPlanning = moduleManager.getModule("TIMRADActionTransport.PathPlanning", "adf.impl.module.algorithm.AStarPathPlanning");
        this.actionExtMove = moduleManager.getExtAction("OptimizedTactics.ExtActionMove", "adf.impl.extaction.OptimizedExtActionMove");
        this.messageManager = new MessageManager();
        
        try {
            this.kernelTime = this.scenarioInfo.getKernelTimesteps();
        } catch (NoSuchConfigOptionException e) {
            this.kernelTime = -1;
        }
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
