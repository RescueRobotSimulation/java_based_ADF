package TIMRAD_2025.extaction;

import adf.core.agent.action.Action;
import adf.core.agent.action.ambulance.ActionLoad;
import adf.core.agent.action.ambulance.ActionRescue;
import adf.core.agent.action.ambulance.ActionUnload;
import adf.core.agent.action.common.ActionMove;
import adf.core.agent.action.common.ActionRest;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.StandardMessagePriority;
import adf.core.agent.communication.standard.bundle.centralized.CommandFire;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.extaction.ExtAction;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;

import com.google.common.collect.Lists;
import rescuecore2.config.NoSuchConfigOptionException;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.standard.entities.StandardEntityConstants.Fieryness;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

import TIMRAD_2025._taskmanager.*;
import TIMRAD_2025._taskmanager.sandbox.*;
import TIMRAD_2025._taskmanager.ConcreteEvaluator.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

public class TIMRADTransport extends ExtAction {
    private PathPlanning pathPlanning;

    // calcSearch用
    private Clustering clustering;
    private boolean stopped;
    private ArrayList<EntityID> unsearchedBuildingIDs;
    private int clusterIndex;
    private int changeClusterCycle;
    protected Random random;
    private ArrayList<Point2D> previousLocations;
    private ArrayList<List<EntityID>> previousPaths;

    private int thresholdRest;
    private int kernelTime;

    private EntityID target;
    private final RIODynamicTaskManager taskManager = new RIODynamicTaskManager();
    private MessageManager messageManager;


    public TIMRADTransport(AgentInfo agentInfo, WorldInfo worldInfo, ScenarioInfo scenarioInfo, ModuleManager moduleManager, DevelopData developData) {
        super(agentInfo, worldInfo, scenarioInfo, moduleManager, developData);
        this.target = null;
        this.thresholdRest = developData.getInteger("ActionTransport.rest", 100);

        switch (scenarioInfo.getMode()) {
            case PRECOMPUTATION_PHASE:
                this.pathPlanning = moduleManager.getModule("ActionTransport.PathPlanning", "RIO2023.algorithm.AstarPathPlanning");
                break;
            case PRECOMPUTED:
                this.pathPlanning = moduleManager.getModule("ActionTransport.PathPlanning", "RIO2023.algorithm.AstarPathPlanning");
                break;
            case NON_PRECOMPUTE:
                this.pathPlanning = moduleManager.getModule("ActionTransport.PathPlanning", "RIO2023.algorithm.AstarPathPlanning");
                break;
        }

        // calcSearch用
        this.clustering = moduleManager.getModule("ActionTransport.Clustering.Ambulance", "RIO2023.algorithm.RioneKmeansPP");
        unsearchedBuildingIDs = new ArrayList<>();
        this.changeClusterCycle = 5;
        this.clusterIndex = 0;
        this.random = new Random();
        this.stopped = false;
        this.previousLocations = new ArrayList<>();
        this.previousPaths = new ArrayList<>();
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
        this.rioSetup();
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
        this.rioSetup();
        return this;
    }

    /**
     * @author Horie(16th)
     * @since 2022.1
     * 事前計算の有無に関係ない共通の初期化事項
     * 動的タスク管理の実現のために切り分けた
     */
    private void rioSetup(){
        /// 1.taskManagerのセットアップ・評価基準登録
        /// discussion : taskManageのupdate()とexecutation()を一度実行すべきか？（編集中）

        this.taskManager.setInfo(this.scenarioInfo, this.worldInfo, this.agentInfo)
                        .setAlgorithm(this.pathPlanning, null)
                        .setTargetKind(
                            StandardEntityURN.CIVILIAN,
                            StandardEntityURN.AMBULANCE_TEAM,
                            StandardEntityURN.FIRE_BRIGADE,
                            StandardEntityURN.POLICE_FORCE,
                            StandardEntityURN.REFUGE
                        );

        this.taskManager.setEvaluator(new RIOEvaluatorBuildingEntrance())
                        .setEvaluator(new RIOEvaluatorAroundCrowd());
                        //.setEvaluator(new RIOERemoveBlocksAroundTheNearestAgent());
                        //.setEvaluator(new RIOCalcTimeToBed());
        //              .setEvaluator(new RIOEvaluatorAroundFire());

    }

    public ExtAction updateInfo(MessageManager messageManager) {
        this.messageManager = messageManager;
        super.updateInfo(messageManager);
            
        if (this.getCountUpdateInfo() >= 2) {
            return this;
        }
        this.pathPlanning.updateInfo(messageManager);

        if (this.unsearchedBuildingIDs.isEmpty()) {
            this.reset();
        }

        // 未探索建物の精査
        List<EntityID> perceivedBuildings = new ArrayList<>();// 見つけた建物
        for (EntityID id : worldInfo.getChanged().getChangedEntities()) {
            StandardEntity se = worldInfo.getEntity(id);
            if (se instanceof Building) {
                perceivedBuildings.add(id);
            }
        }
        for (EntityID pID : perceivedBuildings) {
            //not need to check list contains
            unsearchedBuildingIDs.remove(pID);
        }


        return this;
    }

    @Override
    public ExtAction setTarget(EntityID target) {
        this.target = null; //前回のtarget
        if (target != null) { //今のtarget
            StandardEntity entity = this.worldInfo.getEntity(target);
            if (entity instanceof Human || entity instanceof Area) {
                this.target = target;
                return this;
            }
        }
        return this;
    }


    @Override
    public ExtAction calc() {
        //System.out.println("RIOTransport is working");
        this.result = null;
        AmbulanceTeam agent = (AmbulanceTeam) this.agentInfo.me();
        Human transportHuman = null;
        try {
            transportHuman = this.agentInfo.someoneOnBoard();
        } catch (Exception e) {
            // TODO: handle exception
        }
        if (transportHuman != null) {
            this.result = this.calcUnload(agent, this.pathPlanning, transportHuman, this.target);
            if (this.result != null) {
                return this;
            }
        }
        // ATのいる場所が燃えてる建物なら隣のエリアに移動する
        StandardEntity position = this.worldInfo.getEntity(agentInfo.getPosition());
        if (position instanceof Building && ((Building) position).isOnFire() &&
                (((Building) position).getFierynessEnum() == Fieryness.INFERNO || ((Building) position).getFierynessEnum() == Fieryness.BURNING)) {
            pathPlanning.setFrom(position.getID());
            pathPlanning.setDestination(((Building) position).getNeighbours());
            List<EntityID> path = pathPlanning.calc().getResult();
            if (path != null && path.size() > 0) {
                this.result = calcActionMove(path);  //Buildingからでる（隣のroad（getNeighbor））)
            }
            return this;
        }
        //this.resultが指定されない時
        if (this.needRest(agent)) {
            EntityID areaID = this.convertArea(this.target);  //(convert変換する)targetのposition（areaID）を返す
            ArrayList<EntityID> targets = new ArrayList<>();
            if (areaID != null) {
                targets.add(areaID);
            }
            this.result = this.calcRefugeAction(agent, this.pathPlanning, targets, false);
            if (this.result != null) {
                return this;
            }
        }
        if (this.target != null) {
            this.result = this.calcRescue(agent, this.pathPlanning, this.target);
        }
        return this;
    }

    // 止まってる判定はtrue、止まってなければfalse
    private boolean isStopped(List<EntityID> path1, List<EntityID> path2) {
        if (!(this.agentInfo.getExecutedAction(-1) instanceof ActionMove)){
            return false;
        }
        Human agent = (Human) this.agentInfo.me();
        previousLocations.add(new Point2D(agent.getX(), agent.getY()));//移動するときの場所を記録(0が現在地)

        if (path1 == null || path2 == null) {
            return false;
        }
        if (path1.size() != path2.size()) {
            return false;
        } else {
            for (int i = 0; i < path1.size(); i++) {
                EntityID id1 = path1.get(i);
                EntityID id2 = path2.get(i);
                if (!id1.equals(id2))
                    return false;
            }
        }

        if (previousLocations.size() > 2) {
            return withinRange(previousLocations.get(0), previousLocations.get(1), previousLocations.get(2));
        }
        return false;
    }

    private boolean withinRange(Point2D position1, Point2D position2, Point2D position3) {
        int range = 30000;

        double dist1 = GeometryTools2D.getDistance(position1, position2);
        double dist2 = GeometryTools2D.getDistance(position1, position3);

        return dist1 < range && dist2 < range;

    }

    private void reset() {
        this.unsearchedBuildingIDs.clear();
        this.previousPaths.clear();
        this.previousLocations.clear();

        if ((this.agentInfo.getTime() != 0 && (this.agentInfo.getTime() % this.changeClusterCycle) == 0) || stopped) {
            this.stopped = false;
            this.clusterIndex = random.nextInt(clustering.getClusterNumber());
            this.changeClusterCycle = random.nextInt(16) + 15;//変更

        }
        Collection<StandardEntity> clusterEntities = new ArrayList<>();
        if (clustering != null) {
            clusterEntities.addAll(this.clustering.getClusterEntities(clusterIndex));
        }

        if (!clusterEntities.isEmpty() && clusterEntities.size() > 0) {
            for (StandardEntity entity : clusterEntities) {
                if (entity instanceof Building && entity.getStandardURN() != REFUGE) {
                    this.unsearchedBuildingIDs.add(entity.getID());
                }
            }
        } else {
            this.unsearchedBuildingIDs.addAll(this.worldInfo.getEntityIDsOfType(BUILDING));
        }
    }

    private Action calcActionMove(List<EntityID> path) {
        previousPaths.add(path);
        if (previousPaths.size() < 2 || !isStopped(previousPaths.get(0), previousPaths.get(1))) {
            if (path != null && path.size() > 0) {
                StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
                if (entity instanceof Building) {
                    if (entity.getStandardURN() != StandardEntityURN.REFUGE && ((Building) entity).isOnFire()) {
                        path.remove(path.size() - 1);
                    }
                }
                if (path.size() > 0) {
                    return new ActionMove(path);
                }
            }
            return null;
        }
        return null;
    }


    private Action calcRescue(AmbulanceTeam agent, PathPlanning pathPlanning, EntityID targetID) {
        StandardEntity targetEntity = this.worldInfo.getEntity(targetID);
        if (targetEntity == null) {
            return null;
        }
        EntityID agentPosition = agent.getPosition();
        if (targetEntity instanceof Human) {
            Human human = (Human) targetEntity;
            if (!human.isPositionDefined()) {
                return null;
            }
            if (human.isHPDefined() && human.getHP() == 0) {
                return null;
            }
            EntityID targetPosition = human.getPosition();
            if (!human.isBuriednessDefined() || human.getBuriedness() == 0) {
                if (agentPosition.getValue() == targetPosition.getValue()) {
                    return new ActionLoad(human.getID());
                } else if (this.worldInfo.getPosition(targetID).getStandardURN().getURNId() != StandardEntityURN.REFUGE.getURNId()){
                    List<EntityID> path = pathPlanning.getResult(agentPosition, targetPosition);
                    if (path != null && path.size() > 0) {
                        return calcActionMove(path);
                    }
                }
            } 
            else {
                if (agentPosition.getValue() == targetPosition.getValue()) {
                    // 新ルール下ではRescue「Rest」しちゃう
                    List<Integer> agentIDsValues = new ArrayList<>();
                    for (EntityID entityID : this.worldInfo.getEntityIDsOfType(StandardEntityURN.AMBULANCE_TEAM)) {
                        if (this.worldInfo.getPosition(entityID).getID().getValue() != this.worldInfo.getPosition(targetID).getID().getValue()){
                            continue;
                        }
                        agentIDsValues.add(entityID.getValue());
                    }
                    if (agentIDsValues.isEmpty()) {
                        messageManager.addMessage(new CommandFire(true, StandardMessagePriority.NORMAL, agent.getID(), targetID, CommandFire.ACTION_RESCUE));
                        return new ActionRest();
                    }
                    else if (agent.getID().getValue() == Collections.min(agentIDsValues)) {
                        messageManager.addMessage(new CommandFire(true, StandardMessagePriority.NORMAL, agent.getID(), targetID, CommandFire.ACTION_RESCUE));
                        return new ActionRest();
                    }
                }
                else {
                    List<EntityID> path = pathPlanning.getResult(agentPosition, targetPosition);
                    if (path != null && path.size() > 0) {
                        return calcActionMove(path);
                    }
                }
            }
            return null;
        }
        if (targetEntity instanceof Area) {
            List<EntityID> path = pathPlanning.getResult(agentPosition, targetEntity.getID());
            if (path != null && path.size() > 0) {
                this.result = calcActionMove(path);
            }
        }
        return null;
    }

    private Action calcUnload(AmbulanceTeam agent, PathPlanning pathPlanning, Human transportHuman, EntityID targetID) {
        // 背負っている Human が背負う意味のないものだった場合の処理
        if (transportHuman == null) {
            return null;
        }
        if (transportHuman.isHPDefined() && transportHuman.getHP() == 0) {
            return new ActionUnload();
        }

        EntityID agentPosition = agent.getPosition();
        StandardEntity position = null;
        if (targetID == null || transportHuman.getID().getValue() == targetID.getValue()) {
            position = this.worldInfo.getEntity(agentPosition);
        }

        if (position == null){
            // -- NOTHING_TO_DO -- （ネストを減らすための工夫）
        } else if (position != null && position.getStandardURN() == REFUGE) {
            // 避難所についたので下ろす
             return new ActionUnload();
        } else {
            // 避難所の選択
            pathPlanning.setFrom( agentPosition );

            /**
             * ここに避難所選択の動的タスク管理アルゴリズムを導入する
             */
            // DynamicTaskMangerの評価値付与処理
            this.taskManager.update();
            EntityID targetRefuge = this.taskManager.executation();


            Collection<EntityID> destination = new ArrayList<>(List.of(targetRefuge));
            this.pathPlanning.setDestination(destination);
            //}}
            List<EntityID> path = pathPlanning.calc().getResult();
            if ( path != null && path.size() > 0 ) {
                return new ActionMove( path );
            }
        }
        if (targetID == null) {
            return null;
        }
        StandardEntity targetEntity = this.worldInfo.getEntity(targetID);
        if (targetEntity != null && targetEntity.getStandardURN() == BLOCKADE) {
            Blockade blockade = (Blockade) targetEntity;
            if (blockade.isPositionDefined()) {
                targetEntity = this.worldInfo.getEntity(blockade.getPosition());
            }
        }

        if (targetEntity instanceof Area) {
            if (agentPosition.getValue() == targetID.getValue()) {
                return new ActionUnload();
            }
            pathPlanning.setFrom(agentPosition);
            pathPlanning.setDestination(targetID);
            List<EntityID> path = pathPlanning.calc().getResult();
            if (path != null && path.size() > 0) {
                return calcActionMove(path);
            }

        } else if (targetEntity instanceof Human) {
            Human human = (Human) targetEntity;
            if (human.isPositionDefined()) {
                return calcRefugeAction(agent, pathPlanning, Lists.newArrayList(human.getPosition()), true);
            }

            pathPlanning.setFrom(agentPosition);
            pathPlanning.setDestination(this.worldInfo.getEntityIDsOfType(REFUGE));
            List<EntityID> path = pathPlanning.calc().getResult();
            if (path != null && path.size() > 0) {
                return calcActionMove(path);
            }
        }
        return null;
    }

    private boolean needRest(Human agent) {
        int hp = agent.getHP();
        int damage = agent.getDamage();
        if (hp == 0 || damage == 0) {
            return false;
        }
        int activeTime = (hp / damage) + ((hp % damage) != 0 ? 1 : 0);
        if (this.kernelTime == -1) {
            try {
                this.kernelTime = this.scenarioInfo.getKernelTimesteps();
            } catch (NoSuchConfigOptionException e) {
                this.kernelTime = -1;
            }
        }
        return damage >= this.thresholdRest || (activeTime + this.agentInfo.getTime()) < this.kernelTime;
    }

    private EntityID convertArea(EntityID targetID) {
        StandardEntity entity = this.worldInfo.getEntity(targetID);
        if (entity == null) {
            return null;
        }
        if (entity instanceof Human) {
            Human human = (Human) entity;
            if (human.isPositionDefined()) {
                EntityID position = human.getPosition();
                if (this.worldInfo.getEntity(position) instanceof Area) {
                    return position;
                }
            }
        } else if (entity instanceof Area) {
            return targetID;
        } else if (entity.getStandardURN() == BLOCKADE) {
            Blockade blockade = (Blockade) entity;
            if (blockade.isPositionDefined()) {
                return blockade.getPosition();
            }
        }
        return null;
    }

    private Action calcRefugeAction(Human human, PathPlanning pathPlanning, Collection<EntityID> targets, boolean isUnload) {
        EntityID position = human.getPosition();
        Collection<EntityID> refuges = this.worldInfo.getEntityIDsOfType(StandardEntityURN.REFUGE);
        int size = refuges.size();
        if (refuges.contains(position)) {
            return isUnload ? new ActionUnload() : new ActionRest();
        }
        List<EntityID> firstResult = null;
        while (refuges.size() > 0) {
            pathPlanning.setFrom(position);
            pathPlanning.setDestination(refuges);
            List<EntityID> path = pathPlanning.calc().getResult();
            if (path != null && path.size() > 0) {
                if (firstResult == null) {
                    firstResult = new ArrayList<>(path);
                    if (targets == null || targets.isEmpty()) {
                        break;
                    }
                }
                EntityID refugeID = path.get(path.size() - 1);
                pathPlanning.setFrom(refugeID);
                pathPlanning.setDestination(targets);
                List<EntityID> fromRefugeToTarget = pathPlanning.calc().getResult();
                if (fromRefugeToTarget != null && fromRefugeToTarget.size() > 0) {
                    return calcActionMove(path);
                }
                refuges.remove(refugeID);
                //remove failed
                if (size == refuges.size()) {
                    break;
                }
                size = refuges.size();
            } else {
                break;
            }
        }
        return firstResult != null ? calcActionMove(firstResult) : null;
    }

    private class DistanceSorter implements Comparator<StandardEntity> {
        private StandardEntity reference;
        private WorldInfo worldInfo;

        DistanceSorter(WorldInfo wi, StandardEntity reference) {
            this.reference = reference;
            this.worldInfo = wi;
        }

        public int compare(StandardEntity a, StandardEntity b) {
            int d1 = this.worldInfo.getDistance(this.reference, a);
            int d2 = this.worldInfo.getDistance(this.reference, b);
            return d1 - d2;
        }
    }

}