package TIMRAD_2025.detector;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.MessageUtil;
import adf.core.agent.communication.standard.bundle.centralized.CommandAmbulance;
import adf.core.agent.communication.standard.bundle.centralized.CommandFire;
import adf.core.agent.communication.standard.bundle.information.MessageAmbulanceTeam;
import adf.core.agent.communication.standard.bundle.information.MessageBuilding;
import adf.core.agent.communication.standard.bundle.information.MessageFireBrigade;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.communication.CommunicationMessage;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.complex.BuildingDetector;
import rescuecore2.config.Config;
import rescuecore2.standard.entities.*;
import rescuecore2.standard.entities.StandardEntityConstants.Fieryness;
import rescuecore2.standard.kernel.comms.ChannelCommunicationModel;
import rescuecore2.worldmodel.EntityID;

import java.util.*;
import java.util.List;

import TIMRAD_2025._taskmanager.*;
import TIMRAD_2025._taskmanager.ConcreteEvaluator.*;
import TIMRAD_2025._taskmanager.sandbox.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

public class TIMRADBuildingDetector extends BuildingDetector {
    private EntityID result;
    private Clustering clustering;
    private int maxExtinguishDistance;
    private final RIODynamicTaskManager taskManager = new RIODynamicTaskManager();
    private MessageManager messageManager;

    public TIMRADBuildingDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
        switch (si.getMode()) {
            case PRECOMPUTATION_PHASE:
                this.clustering = moduleManager.getModule("BuildingDetector.Clustering", "RIO2023.algorithm.RioneKmeansPP");
                break;
            case PRECOMPUTED:
                this.clustering = moduleManager.getModule("BuildingDetector.Clustering", "RIO2023.algorithm.RioneKmeansPP");
                break;
            case NON_PRECOMPUTE:
                this.clustering = moduleManager.getModule("BuildingDetector.Clustering", "RIO2023.algorithm.RioneKmeansPP");
                break;
        }
        registerModule(this.clustering);

        this.maxExtinguishDistance = scenarioInfo.getFireExtinguishMaxDistance();
    }


    @Override
    public BuildingDetector updateInfo(MessageManager messageManager) {

        // 視界情報の更新
        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        changedEntities.add(this.agentInfo.me().getID());

        for (CommunicationMessage message : messageManager.getReceivedMessageList()) {
            if (message instanceof MessageFireBrigade) {
                MessageFireBrigade messageFB = (MessageFireBrigade) message;
            } else if (message instanceof MessageBuilding) {
                MessageBuilding mb = (MessageBuilding) message;
                if (!changedEntities.contains(mb.getBuildingID())) {
                    MessageUtil.reflectMessage(this.worldInfo, mb);
                }
            }
        }

        this.messageManager = messageManager;
        super.updateInfo(messageManager);
        
        if (this.getCountUpdateInfo() >= 2) {
            return this;
        }

        return this;
    }

    @Override
    public BuildingDetector calc() {
        System.out.println("BUildingDetector is working!");
        // DynamicTaskMangerの評価値付与処理
        if (this.result != null) {
            return this;
        }
        this.taskManager.update();

        // ↓↓2021ルール対応後にコメントアウト解除
        this.result = this.taskManager.executation();
        // ↑↑2021ルール対応後にコメントアウト解除
        return this;
    }




    @Override
    public EntityID getTarget() {
        return this.result;
    }

    @Override
    public BuildingDetector precompute(PrecomputeData precomputeData) {
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
    public BuildingDetector resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if (this.getCountPrecompute() >= 2) {
            return this;
        }
        this.rioSetUp();
        return this;
    }

    /**
     * 事前計算がない場合の初期化処理
     */
    @Override
    public BuildingDetector preparate() {
        super.preparate();
        if (this.getCountPrecompute() >= 2) {
            return this;
        }
        this.rioSetUp();
        return this;
    }

    /**
     * @author Horie(16th)
     * @since 2021.3
     * 事前計算の有無に関係ない共通の初期化事項
     * 動的タスク管理の実現のために切り分けた
     */
    public void rioSetUp(){
        /// 1.taskManagerのセットアップ・評価基準の登録
        this.taskManager.setInfo(this.scenarioInfo, this.worldInfo, this.agentInfo)
                        .setAlgorithm(null, this.clustering)
                        .setTargetKind(StandardEntityURN.ROAD);

        this.taskManager.setEvaluator(new RIOEvaluatorTime());
                        // .setEvaluator(new RIOEvaluatorBeforeHand())
                        // .setEvaluator(new RIOEvaluatorClusterBuildings())
                        // .setEvaluator(new RIOEvaluatorFireInSight())
                        // .setEvaluator(new RIOEvaluatorOnlyDistance());

        /// 3.taskManagerを一度実行？（編集中）
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

    private class TempSorter implements Comparator<Building> {
        public int compare(Building a, Building b) {
            int d1 = a.isTemperatureDefined() ? a.getTemperature() : 0;
            int d2 = b.isTemperatureDefined() ? b.getTemperature() : 0;
            return d1 - d2;
        }
    }



    //「温度が低く」かつ「近くの建物が多い」建物を優先
    private EntityID calcPriority() {
        FireBrigade fireBrigade = (FireBrigade) this.agentInfo.me();
        ArrayList<StandardEntity> allFB = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE));//全ての「FB」
        ArrayList<StandardEntity> buildings = new ArrayList<>(this.worldInfo.getEntitiesOfType(BUILDING));//全ての「建物」
        ArrayList<Building> fireBuildings = new ArrayList<>();//全ての「燃えている建物」
        ArrayList<Building> BuildingsNearFire = new ArrayList<>();//全ての「燃えている建物付近の建物」
        for (StandardEntity entityB : buildings) {
            if (entityB instanceof Building) {
                Building building = (Building) entityB;
                //「建物」が燃えている場合
                if (building.isOnFire()) {
                    //全ての「燃えている建物」を特定
                    fireBuildings.add(building);
                }
            }
        }

        for (StandardEntity entityFire : fireBuildings) {
            for (StandardEntity entityBuilding : buildings) {
                if (entityBuilding instanceof Building) {
                    Building building = (Building) entityBuilding;
                    //「建物」が燃えておらず、「燃えている建物」付近にいる場合
                    if (!building.isOnFire() &&
                            worldInfo.getDistance(entityBuilding, entityFire) <= maxExtinguishDistance/2) {
                        //全ての「燃えている建物付近の建物」を特定
                        BuildingsNearFire.add(building);
                    }
                }
            }
        }
        double min = Integer.MAX_VALUE; //無限
        double pri;
        EntityID target;
        int nfNumber = 0; //「燃えている建物付近の建物」の数

        for (StandardEntity entityFire : fireBuildings) {
            for (StandardEntity entityNF : BuildingsNearFire) {
                if(worldInfo.getDistance(entityFire, entityNF) <= maxExtinguishDistance/2) {
                    nfNumber++;
                }

                pri = (double)((Building)entityFire).getTemperature()/nfNumber; //優先基準のパラメータ=温度/周りの建物の数
                if (pri < min) { //パラメータの最小値を更新
                	    min = pri;
                	    target = entityFire.getID();
                	    return target;
                 }
             }
         }

        return null;
    }

}
