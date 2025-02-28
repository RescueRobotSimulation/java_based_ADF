package TIMRAD_2025._taskmanager.sandbox;

import adf.core.agent.Agent;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.MessageUtil;
import adf.core.agent.communication.standard.bundle.StandardMessagePriority;
import adf.core.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.core.agent.communication.standard.bundle.information.MessageAmbulanceTeam;
import adf.core.agent.communication.standard.bundle.information.MessageFireBrigade;
import adf.core.agent.communication.standard.bundle.information.MessageCivilian;
import adf.core.agent.communication.standard.bundle.information.MessagePoliceForce;
import adf.core.agent.communication.standard.bundle.information.MessageBuilding;
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

/**
 * @author Nomoto(17th)
 * @since 2022 PFが除去する瓦礫の優先度を計算
        　PFから最も近い(埋まっている)Humanまでの道にある瓦礫の優先度を高く
        　周りに埋まっている他のHumanがいればより高く
 */
public class RIOERemoveBlocksAroundTheNearestAgent extends RIOEDefault {
    private final double weightCrowd = 1000; // attention : マジックナンバー
    private double clearDistance;
    
    // ArrayListではなく削除・検索が高速な(Hash)Setを使う
    private Collection<EntityID> civilians; // 全ての「市民」
    private Collection<EntityID> ambulanceTeams; // 全ての「AT」
    private Collection<EntityID> fireBrigades; // 全ての「FB」
    private Collection<EntityID> policeForce; // 全ての「PF」
    private Collection<EntityID> allBlockades; // 全てのガレキ
    private Map<EntityID, Double> BlockadesPriority = new HashMap<>(); //各瓦礫の優先度
    private MessageManager messagemanager = new MessageManager();
    private int helpCount;

    @Override
    protected IRIOEvaluator init(){
        this.clearDistance = this.scenarioInfo.getClearRepairDistance();
        // 全ての「市民」の情報を取得してリストに加える
        civilians = new HashSet<>(worldInfo.getEntityIDsOfType(CIVILIAN));
        // 全ての「AT」の情報を取得してリストに加える
        ambulanceTeams = new HashSet<>(worldInfo.getEntityIDsOfType(AMBULANCE_TEAM));
        // 全ての「PF」の情報を取得してリストに加える
        policeForce = new HashSet<>(worldInfo.getEntityIDsOfType(FIRE_BRIGADE));
        // 全ての「FB」の情報を取得してリストに加える
        fireBrigades = new HashSet<>(worldInfo.getEntityIDsOfType(POLICE_FORCE));
        //全てのガレキの情報を取得してリストに加える
        allBlockades = new HashSet<>(worldInfo.getEntityIDsOfType(BLOCKADE));
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

    /// summary : 最も悪い値 = 0
    @Override
    protected double worstPriority() {
        return 0;
    }

    /// summary : 最も不安定 = 0
    @Override
    protected double bestUnstableness() {
        return 0;
    }

    /// summary : 最も安定 = 1（使わないけど）
    @Override
    protected double worstUnstableness() {
        return 0;
    }

    /// ---- 各種計算 ----
    /// summary : 各サイクルごとの事前処理    
    @Override
    public IRIOEvaluator update(){
        this.civilians = new HashSet<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.CIVILIAN)); // 全ての「市民」
        this.ambulanceTeams = new HashSet<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.AMBULANCE_TEAM)); // 全ての「AT」
        this.fireBrigades = new HashSet<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.FIRE_BRIGADE)); // 全ての「FB」
        //全てのガレキの情報
        //calcで「自分の現在地にあるガレキ」を取ってくるので必要ないかも？
        //this.allBlockades = new HashSet<>(this.worldInfo.getBlockades(area));

        
        //叫び声のメッセージカウント(0でなければ叫び声が聞こえている、人が埋まっている)
        this.helpCount = messagemanager.getHeardAgentHelpCount();
        if(this.helpCount > 0){
            System.out.println(this.helpCount);
        }
        return this;
    }
 
 
    /// summary : 優先度計算(編集中), 不安定度計算(ココでは何もしない)
    @Override
    public RIODynamicTaskKey calc(EntityID evaluateeID) {
        double priority = this.getPriority(evaluateeID);
        double unstableness = 0;
        return new RIODynamicTaskKey(null, priority, unstableness);       
    }

    private double getPriority(EntityID evaluateeID){
        double priority = 0;
        Human policeForce = (Human)this.agentInfo.me();

        //「自分の現在地」を取得
		EntityID humanPositionID = policeForce.getPosition();
		// 変数 humanPositionID をStandardEntity型に変換
		Area humanPosition = (Area)this.worldInfo.getEntity(humanPositionID);
        
        if(! humanPosition.isBlockadesDefined()){
            return priority;
        }

        //全ての「自分の現在地にあるガレキ」の情報を取得してリストに加える
        List<EntityID> areaInBlockades = (humanPosition).getBlockades();

        
        for(EntityID blockadesID :areaInBlockades){
            //叫び声を聞くと数値がプラスされるようにしたい
            if(this.helpCount > 0){
                priority += 1;
            }
            //叫び声のメッセージカウントをリセット
            //messagemanager.refresh();
        } 
        if(priority > 0){
            System.out.println(priority);
        }

        return priority;
    }

}
