package TIMRAD_2025.detector;

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
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

// 17期リーダー：Java17への移行の際にパッケージが読み込まれないため、不要そうなので消しました
// import org.apache.commons.math3.stat.descriptive.StorelessUnivariateStatistic;

import TIMRAD_2025._taskmanager.*;
import TIMRAD_2025._taskmanager.sandbox.*;
import TIMRAD_2025._taskmanager.ConcreteEvaluator.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

public class TIMRADRoadDetector extends RoadDetector {
    private Set<EntityID> targetAreas;
    private Set<EntityID> priorityRoads;

    private PathPlanning pathPlanning;
    private int clearDistance; // 「瓦礫」を除去する範囲
    private EntityID result;

    // checkDeadEnd
    private EntityID cDEresult; // checkDeadEndで得たresult
    private EntityID cDElog; // checkDeadEndで得たresultのうち, nullじゃなかった最後のもの
    private StandardEntity cDETarget = null;
    private List<StandardEntity> deadendPoints = new ArrayList<StandardEntity>(Arrays.asList(null, null, null));
    private final RIODynamicTaskManager taskManager = new RIODynamicTaskManager();
    private MessageManager messageManager;

    /**
     * @author Horie(16th)
     * @since 2020.10.-
     * summary : 動的なtarget選択のために, 評価基準内部クラスを実装
     * (interfaceはDynamicTaskEvaluator)
     */


    public TIMRADRoadDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager,
            DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
        this.clearDistance = si.getClearRepairDistance();

        switch (scenarioInfo.getMode()) {
            case PRECOMPUTATION_PHASE:
                this.pathPlanning = moduleManager.getModule("SampleRoadDetector.PathPlanning",
                        "adf.core.component.module.algorithm.PathPlanning");
                break;
            case PRECOMPUTED:
                this.pathPlanning = moduleManager.getModule("SampleRoadDetector.PathPlanning",
                        "adf.core.component.module.algorithm.PathPlanning");
                break;
            case NON_PRECOMPUTE:
                this.pathPlanning = moduleManager.getModule("SampleRoadDetector.PathPlanning",
                        "adf.core.component.module.algorithm.PathPlanning");
                break;
        }
        registerModule(this.pathPlanning);
        this.result = null;
    }

    /**
     * @author Horie(16th)(DynamicTaskManager)
     * メインの処理（外部から呼び出す部分）
     *
     */
    @Override
    public RoadDetector calc() {
        //System.out.println("RoadDetector is working!");
        // DynamicTaskMangerの評価値付与処理
        
        if (this.result != null) {
            return this;
        }
        this.result = this.taskManager.executation();
        
        this.taskManager.update();

        EntityID positionID = this.agentInfo.getPosition();

        // 注記：targetAreas はサンプルコードで通信を想定して使われている
        // （2020時点でRi-oneでは未使用）
        /*
        if (this.targetAreas.contains(positionID)) {
            this.result = positionID;
            return this;
        }*/


        List<EntityID> removeList = new ArrayList<>(this.priorityRoads.size());
        for (EntityID id : this.priorityRoads) {
            if (!this.targetAreas.contains(id)) {
                removeList.add(id);
            }
        }
        this.priorityRoads.removeAll(removeList);
        if (this.priorityRoads.size() > 0) {
            this.pathPlanning.setFrom(positionID);
            this.pathPlanning.setDestination(this.targetAreas);
            List<EntityID> path = this.pathPlanning.calc().getResult();
            if (path != null && path.size() > 0) {
                this.result = path.get(path.size() - 1);
            }
            return this;
        }

        this.pathPlanning.setFrom(positionID);
        this.pathPlanning.setDestination(this.targetAreas);
        List<EntityID> path = this.pathPlanning.calc().getResult();
        if (path != null && path.size() > 0) {
            this.result = path.get(path.size() - 1);
        }
        //池上 追加
        if(!((Road)this.worldInfo.getEntity(result)).isBlockadesDefined() || ((Road)this.worldInfo.getEntity(result)).getBlockades().isEmpty()){
            this.result = null;
        }

        return this;
    }


    @Override
    public EntityID getTarget() {
        if (this.result != null){
            if(!((Road)this.worldInfo.getEntity(result)).isBlockadesDefined() || ((Road)this.worldInfo.getEntity(result)).getBlockades().isEmpty()){
                this.result = null;
            }
        }
        // if (this.worldInfo.getEntity(result) instanceof Area area){
        //     if (area.getBlockades() != null && !area.getBlockades().isEmpty()){
        //         int[] blockadesApexs;
        //         int[] areaApexs = area.getApexList();
        //         System.out.println(areaApexs);
        //         int sumBlockadesArea = 0;
        //         for (EntityID blockadeID : area.getBlockades()){
        //             blockadesApexs = ((Blockade)this.worldInfo.getEntity(blockadeID)).getApexes();
        //             sumBlockadesArea += Math.abs((blockadesApexs[0]-blockadesApexs[2])*(blockadesApexs[1]-blockadesApexs[3]));
        //         }
        //         System.out.println(sumBlockadesArea/Math.abs((areaApexs[0]-areaApexs[2])*(areaApexs[1]-areaApexs[3])));

        //         if(sumBlockadesArea/Math.abs((areaApexs[0]-areaApexs[2])*(areaApexs[1]-areaApexs[3]))*100 < 5) {
        //             this.result = null;
        //         }
        //     }
        //     else {
        //         this.result = null;
        //     }
        // }
        return this.result;
    }

    /**
     * 事前計算がある場合の初期化処理の準備
     */
    @Override
    public RoadDetector precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        if (this.getCountPrecompute() >= 2) {
            return this;
        }
        return this;
    }

    /**
     * 事前計算がある場合の初期化処理
     */
    @Override
    public RoadDetector resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if (this.getCountResume() >= 2) {
            return this;
        }
        this.rioSetUp();
        return this;
    }

    /**
     * 事前計算がない場合の初期化処理
     */
    @Override
    public RoadDetector preparate() {
        super.preparate();
        if (this.getCountPreparate() >= 2) {
            return this;
        }
        this.rioSetUp();
        return this;
    }

    /**
     * @author Horie(16th)
     * @since 2020.10
     * 事前計算の有無に関係ない共通の初期化事項
     * 動的タスク管理の実現のために切り分けた
     */
    private void rioSetUp(){
        /// 1.taskManagerのセットアップ・評価基準登録
        /// discussion : taskManageのupdate()とexecutation()を一度実行すべきか？（編集中）

        this.taskManager.setInfo(this.scenarioInfo, this.worldInfo, this.agentInfo)
                        .setAlgorithm(this.pathPlanning, null)
                        .setTargetKind(StandardEntityURN.ROAD);

        this.taskManager.setEvaluator(new RIOEvaluatorBuildingEntrance())
                        .setEvaluator(new RIOEvaluatorAroundCrowd())
                        .setEvaluator(new RIOEvaluatorAroundFire())
                        .setEvaluator(new RIOERemoveBlocksAroundTheNearestAgent());

        /// 3.一番最初の探索
        //以下, sampleコードそのまま（今後削除）
        this.targetAreas = new HashSet<>();
        for (StandardEntity e : this.worldInfo.getEntitiesOfType(REFUGE, BUILDING, GAS_STATION)) {
            for (EntityID id : ((Building) e).getNeighbours()) {
                StandardEntity neighbour = this.worldInfo.getEntity(id);
                if (neighbour instanceof Road) {
                    this.targetAreas.add(id);
                }
            }
        }
        this.priorityRoads = new HashSet<>();
        for (StandardEntity e : this.worldInfo.getEntitiesOfType(REFUGE)) {
            for (EntityID id : ((Building) e).getNeighbours()) {
                StandardEntity neighbour = this.worldInfo.getEntity(id);
                if (neighbour instanceof Road) {
                    this.priorityRoads.add(id);
                }
            }
        }
    }

    @Override
    public RoadDetector updateInfo(MessageManager messageManager) {
        this.messageManager = messageManager;
        super.updateInfo(messageManager);
        this.taskManager.setMessageManager(messageManager);
        if (this.getCountUpdateInfo() >= 2) {
            return this;
        }
        if (this.result != null) {
            if (this.agentInfo.getPosition().equals(this.result)) {
                StandardEntity entity = this.worldInfo.getEntity(this.result);
                if (entity instanceof Building) {
                    this.result = null;
                } else if (entity instanceof Road) {
                    Road road = (Road) entity;
                    if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) {
                        this.targetAreas.remove(this.result);
                        this.result = null;
                    }
                }
            }
        }
        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        for (CommunicationMessage message : messageManager.getReceivedMessageList()) {
            Class<? extends CommunicationMessage> messageClass = message.getClass();
            // if (messageClass == MessageAmbulanceTeam.class) {
            //     this.reflectMessage((MessageAmbulanceTeam) message);
            // } else if (messageClass == MessageFireBrigade.class) {
            //     this.reflectMessage((MessageFireBrigade) message);
            if (messageClass == MessageRoad.class) {
                 this.reflectMessage((MessageRoad) message, changedEntities);
            } 
            //else if (messageClass == MessagePoliceForce.class) {
            //     this.reflectMessage((MessagePoliceForce) message);
            // } else if (messageClass == CommandPolice.class) {
            //     this.reflectMessage((CommandPolice) message);
            // }
        }
        for (EntityID id : this.worldInfo.getChanged().getChangedEntities()) {
            StandardEntity entity = this.worldInfo.getEntity(id);
            if (entity instanceof Road) {
                Road road = (Road) entity;
                if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) {
                    this.targetAreas.remove(id);
                }
            }
        }
        return this;
    }

    private void reflectMessage(MessageRoad messageRoad, Collection<EntityID> changedEntities) {
        if (messageRoad.isBlockadeDefined() && !changedEntities.contains(messageRoad.getBlockadeID())) {
            MessageUtil.reflectMessage(this.worldInfo, messageRoad);
        }
        if (messageRoad.isPassable()) {
            this.targetAreas.remove(messageRoad.getRoadID());
        }
    }

    private void reflectMessage(MessageAmbulanceTeam messageAmbulanceTeam) {
        if (messageAmbulanceTeam.getPosition() == null) {
            return;
        }
        if (messageAmbulanceTeam.getAction() == MessageAmbulanceTeam.ACTION_RESCUE) {
            StandardEntity position = this.worldInfo.getEntity(messageAmbulanceTeam.getPosition());
            if (position != null && position instanceof Building) {
                this.targetAreas.removeAll(((Building) position).getNeighbours());
            }
        } else if (messageAmbulanceTeam.getAction() == MessageAmbulanceTeam.ACTION_LOAD) {
            StandardEntity position = this.worldInfo.getEntity(messageAmbulanceTeam.getPosition());
            if (position != null && position instanceof Building) {
                this.targetAreas.removeAll(((Building) position).getNeighbours());
            }
        } else if (messageAmbulanceTeam.getAction() == MessageAmbulanceTeam.ACTION_MOVE) {
            if (messageAmbulanceTeam.getTargetID() == null) {
                return;
            }
            StandardEntity target = this.worldInfo.getEntity(messageAmbulanceTeam.getTargetID());
            if (target instanceof Building) {
                for (EntityID id : ((Building) target).getNeighbours()) {
                    StandardEntity neighbour = this.worldInfo.getEntity(id);
                    if (neighbour instanceof Road) {
                        this.priorityRoads.add(id);
                    }
                }
            } else if (target instanceof Human) {
                Human human = (Human) target;
                if (human.isPositionDefined()) {
                    StandardEntity position = this.worldInfo.getPosition(human);
                    if (position instanceof Building) {
                        for (EntityID id : ((Building) position).getNeighbours()) {
                            StandardEntity neighbour = this.worldInfo.getEntity(id);
                            if (neighbour instanceof Road) {
                                this.priorityRoads.add(id);
                            }
                        }
                    }
                }
            }
        }
    }

    private void reflectMessage(MessageFireBrigade messageFireBrigade) {
        if (messageFireBrigade.getTargetID() == null) {
            return;
        }
        if (messageFireBrigade.getAction() == MessageFireBrigade.ACTION_REFILL) {
            StandardEntity target = this.worldInfo.getEntity(messageFireBrigade.getTargetID());
            if (target instanceof Building) {
                for (EntityID id : ((Building) target).getNeighbours()) {
                    StandardEntity neighbour = this.worldInfo.getEntity(id);
                    if (neighbour instanceof Road) {
                        this.priorityRoads.add(id);
                    }
                }
            } else if (target.getStandardURN() == HYDRANT) {
                this.priorityRoads.add(target.getID());
                this.targetAreas.add(target.getID());
            }
        }
    }

    private void reflectMessage(MessagePoliceForce messagePoliceForce) {
        if (messagePoliceForce.getAction() == MessagePoliceForce.ACTION_CLEAR) {
            if (messagePoliceForce.getAgentID().getValue() != this.agentInfo.getID().getValue()) {
                if (messagePoliceForce.isTargetDefined()) {
                    EntityID targetID = messagePoliceForce.getTargetID();
                    if (targetID == null) {
                        return;
                    }
                    StandardEntity entity = this.worldInfo.getEntity(targetID);
                    if (entity == null) {
                        return;
                    }

                    if (entity instanceof Area) {
                        this.targetAreas.remove(targetID);
                        if (this.result != null && this.result.getValue() == targetID.getValue()) {
                            if (this.agentInfo.getID().getValue() < messagePoliceForce.getAgentID().getValue()) {
                                this.result = null;
                            }
                        }
                    } else if (entity.getStandardURN() == BLOCKADE) {
                        EntityID position = ((Blockade) entity).getPosition();
                        this.targetAreas.remove(position);
                        if (this.result != null && this.result.getValue() == position.getValue()) {
                            if (this.agentInfo.getID().getValue() < messagePoliceForce.getAgentID().getValue()) {
                                this.result = null;
                            }
                        }
                    }

                }
            }
        }
    }

    private void reflectMessage(CommandPolice commandPolice) {
        boolean flag = false;
        if (commandPolice.isToIDDefined() && this.agentInfo.getID().getValue() == commandPolice.getToID().getValue()) {
            flag = true;
        } else if (commandPolice.isBroadcast()) {
            flag = true;
        }
        if (flag && commandPolice.getAction() == CommandPolice.ACTION_CLEAR) {
            if (commandPolice.getTargetID() == null) {
                return;
            }
            StandardEntity target = this.worldInfo.getEntity(commandPolice.getTargetID());
            if (target instanceof Area) {
                this.priorityRoads.add(target.getID());
                this.targetAreas.add(target.getID());
            } else if (target.getStandardURN() == BLOCKADE) {
                Blockade blockade = (Blockade) target;
                if (blockade.isPositionDefined()) {
                    this.priorityRoads.add(blockade.getPosition());
                    this.targetAreas.add(blockade.getPosition());
                }
            }
        }
    }

    /***
     * @author Horie(16th)
     * 優先啓開;
     * マジックナンバー要考察 3s連続で人が溜まってるとこを「立ち往生」と判定し,
     * 一番多いところをtargetにする
     */
    /*
    private EntityID checkDeadEnd() {
        // Human myself = (Human) this.agentInfo.me();
        boolean isSuffered = false; // 選択したHumanの周りにがれきがあるか？
        int botheredHuman = 0; // 道に集まっている（埋まっているかもしれない）人の数
        int maxDeadendHuman = 0;
        StandardEntity tmpTarget = cDETarget;
        final Collection<EntityID> allHuman = this.worldInfo.getEntityIDsOfType(StandardEntityURN.AMBULANCE_TEAM,
                StandardEntityURN.FIRE_BRIGADE);
        final Collection<EntityID> allRoad = this.worldInfo.getEntityIDsOfType(StandardEntityURN.ROAD);
        List<EntityID> neighborRoads; // ある道路+その道路の周辺道路
        cDETarget = null;
        this.deadendPoints.remove(0);
        this.deadendPoints.add(null);
        for (EntityID tmpRoadID : allRoad) {
            StandardEntity tmpRoad = this.worldInfo.getEntity(tmpRoadID);
            if (((Road) tmpRoad).isBlockadesDefined() && ((Road) tmpRoad).getBlockades().size() > 0) { // 瓦礫に埋まった道なら
                botheredHuman = 0;
                neighborRoads = ((Area) tmpRoad).getNeighbours();
                neighborRoads.add(tmpRoadID);
                for (EntityID tmpHumanID : allHuman) { // tmpHuman が tmpRoad 上, もしくはその周りにいるかどうか
                    StandardEntity tmpHuman = this.worldInfo.getEntity(tmpHumanID);
                    for (EntityID tmpNeighborId : neighborRoads) { // 周りの道路についてチェック
                        isSuffered = (this.worldInfo.getEntity(tmpNeighborId) == (tmpRoad));
                        if (isSuffered)
                            break;
                    }
                    if (isSuffered) { // tmpHumanの近くに瓦礫があった場合
                        cDETarget = tmpRoad;
                        botheredHuman++;
                    }
                }
                if (botheredHuman > 0) {
                    if (cDETarget != null && cDETarget == this.deadendPoints.get(0)
                            && this.deadendPoints.get(0) == this.deadendPoints.get(1)) { // [Magic Number]3s連続で選ばれたら
                        this.deadendPoints.set(2, cDETarget);
                        return cDETarget.getID();
                    } else if (maxDeadendHuman < botheredHuman) { // 連続で選ばれていない場合は, 人の数で候補を検討
                        maxDeadendHuman = botheredHuman;
                        this.deadendPoints.set(2, cDETarget); // 候補に追加（決定ではない）
                    }
                }
            }
        }
        return null;
    }
    */

    /**
     * @author Horie(16th)
     *         <p>
     *         AT/FBとの位置関係から優先度を付与する(活用法含め検討中)
     */
    /*
    private void botheringPriority(final StandardEntityURN AGENT_TYPE, final StandardEntityURN DESTINATION_TYPE) {
        final List<EntityID> agents = new ArrayList<>(this.worldInfo.getEntityIDsOfType(AGENT_TYPE));
        final List<EntityID> destinations = new ArrayList<>(this.worldInfo.getEntityIDsOfType(DESTINATION_TYPE));
        final List<EntityID> roads = new ArrayList<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.ROAD));
        for (EntityID roadID : roads) {
            Road nowRoad = ((Road) (this.worldInfo.getEntity(roadID)));
            // 瓦礫がなければカウントしない
            if (!nowRoad.isBlockadesDefined())
                break;
            for (EntityID agentID : agents) {
                for (EntityID destID : destinations) {
                    int value = this.worldInfo.getDistance(agentID, destID) - this.worldInfo.getDistance(roadID, destID)
                            - this.worldInfo.getDistance(agentID, roadID);
                }
            }
            // valueが評価値だが活用法は検討中
        }
    }
    */
}