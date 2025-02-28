package TIMRAD_2025._taskmanager.ConcreteEvaluator;
/*
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

import static rescuecore2.standard.entities.StandardEntityURN.*;

import RIO2023._taskmanager.*;

public class RIOEvaluatorManhattanAT implements RIODynamicTaskEvaluator{
        // 評価対象（target候補）        

        private WorldInfo worldInfo;
        private EntityID myself;
        private EntityID evaluateeID = null;
        private PathPlanning pathPlanning;

        public RIOEvaluatorManhattanAT(Agent myself, EntityID evaluateeID){
            this.worldInfo = myself.worldInfo;
            setEvaluator(myself.getID(), evaluateeID); // 要編集
        }
    
        // 優先度設定のためのパラメータ設定
        @Override
        public RIODynamicTaskEvaluator setEvaluator(EntityID myself, EntityID evaluateeID){
            this.myself = myself;
            this.evaluateeID = evaluateeID;
            return (RIODynamicTaskEvaluator)this;
        }

        // パラメータを設定する関数
        @Override
        public double getPriority(){
            double priority = 0;

            final Collection<EntityID> agents = Collections.unmodifiableCollection(this.worldInfo.getEntityIDsOfType(AMBULANCE_TEAM));
            final Collection<EntityID> destinations = Collections.unmodifiableCollection(this.worldInfo.getEntityIDsOfType(CIVILIAN));
            final Collection<EntityID> roads = Collections.unmodifiableCollection(this.worldInfo.getEntityIDsOfType(StandardEntityURN.ROAD));
    
            for (EntityID roadID : roads) {
                Road nowRoad = ((Road) (this.worldInfo.getEntity(roadID)));
                // 瓦礫がなければカウントしない
                if (!nowRoad.isBlockadesDefined()) continue;
    
                for (EntityID agentID : agents) {
                    for (EntityID destID : destinations) {
                        // ３点間それぞれのパスを取得する
                        //// pathPlanningについて要検討
                        List<EntityID> agentDestPath = pathPlanning.setFrom(agentID).setDestination(destID).calc()
                                .getResult();
                        List<EntityID> destRoadPath = pathPlanning.setFrom(agentID).setDestination(destID).calc()
                                .getResult();
                        List<EntityID> roadAgentPath = pathPlanning.setFrom(agentID).setDestination(destID).calc()
                                .getResult();
                                                
                        priority += destRoadPath.size() + roadAgentPath.size() - agentDestPath.size(); 
                    }
                }
            }
            return priority;
        }

        // 不安定度を返す関数 ////編集中
        @Override
        public double getUnstableness(){
            return 0;
        }
    }

*/