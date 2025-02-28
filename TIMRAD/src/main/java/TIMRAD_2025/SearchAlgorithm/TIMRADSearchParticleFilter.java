package TIMRAD_2025.SearchAlgorithm;

import static rescuecore2.standard.entities.StandardEntityURN.AMBULANCE_CENTRE;
import static rescuecore2.standard.entities.StandardEntityURN.AMBULANCE_TEAM;
import static rescuecore2.standard.entities.StandardEntityURN.BUILDING;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_BRIGADE;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_STATION;
import static rescuecore2.standard.entities.StandardEntityURN.GAS_STATION;
import static rescuecore2.standard.entities.StandardEntityURN.POLICE_FORCE;
import static rescuecore2.standard.entities.StandardEntityURN.POLICE_OFFICE;
import static rescuecore2.standard.entities.StandardEntityURN.REFUGE;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.StandardCommunicationModule;
import adf.core.component.communication.CommunicationMessage;
import adf.core.agent.communication.standard.bundle.MessageUtil;
import adf.core.agent.communication.standard.bundle.StandardMessage;
import adf.core.agent.communication.standard.bundle.centralized.CommandFire;
import adf.core.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import adf.core.component.module.complex.Search;
import rescuecore2.messages.MessageComponent;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.components.StandardAgent;
import rescuecore2.standard.entities.*;
import adf.core.debug.DefaultLogger;
import firesimulator.world.AmbulanceTeam;
import firesimulator.world.Civilian;
import rescuecore.commands.Update;

import org.apache.log4j.Logger;

import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;

import java.util.*;
import java.lang.Math;
import java.awt.geom.Rectangle2D;

 /**
  * @author Ikegami(18th)
  * @since  2023.3.20
  * @brief  粒子フィルタを用いて,叫んでいる市民の位置を探す。
  */

public class TIMRADSearchParticleFilter extends TIMRADDefaultSearch{

    private Pair<Integer, Integer> middlePoint;
    private Map<Pair<Integer, Integer>, Float> particleMap = new HashMap<>();
    private int previousNumberOfHearedHelp = -1; // -1 = NULL
    public  int cycleOfWorking = 0;
    public  boolean WORKING = false;

    @Override
    public TIMRADSearchParticleFilter updateInfo(MessageManager mm){
        super.updateInfo(mm);
        this.messageManager = mm;
        //WORKINGがtrueなら+1 : falseなら0を代入する
        cycleOfWorking = (WORKING) ? cycleOfWorking + 1 : 0;
        return this;
    }

    public void init() {
        WORKING = true;
        this.initMap();
    }

    /**
     * 次のサイクルへ市民の叫び声の数を送る
     * @param hardHelpCount
     */
    public void setPreviousHeardHelp(int hardHelpCount) {
        this.previousNumberOfHearedHelp = hardHelpCount;
    }


    /**
     * 市民がいる可能性の最も高いEntityIDを返す
     * そのEntity内のweightの合計を「市民がいる確率」とする
     * @return EntityID
     */
    public EntityID getTarget() {
        EntityID result = null;
        Set<EntityID> allEntityID = new HashSet<>();
        List<EntityID> path = new ArrayList<>();
        Map<EntityID, Float> resultMap = new HashMap<>();
        boolean xInEntity = false;
        boolean yInEntity = false;
        double averageWeight = 0;
        int perceptionReach = scenarioInfo.getRawConfig().getIntValue("comms.channels.0.range") + scenarioInfo.getPerceptionLosMaxDistance();
        int[] apex;

        //できるだけ広範囲のEntityIDを取得するために過去に見たEntityを取得
        for (int i = 0; i < agentInfo.getTime(); i++) {
            if (Math.pow(this.worldInfo.getLocation(i, agentInfo.me()).first()-this.middlePoint.first(), 2) + Math.pow(this.worldInfo.getLocation(i, agentInfo.me()).second()-this.middlePoint.second(), 2) <= perceptionReach*perceptionReach) {
                if (this.agentInfo.me().getStandardURN().getURNId() == StandardEntityURN.POLICE_FORCE.getURNId()) {
                    allEntityID.addAll(this.worldInfo.getEntityIDsOfType(i, StandardEntityURN.ROAD));
                }
                else {
                    allEntityID.addAll(this.worldInfo.getEntityIDsOfType(i, StandardEntityURN.BUILDING, StandardEntityURN.ROAD));
                }
            }
        }
        //そのParticleがどのEntityに所属しているか,AgentがそのEntityに到達できるのかを調べる
        for (Map.Entry<Pair<Integer, Integer>, Float> entry : this.getParticleMap().entrySet()) {
            for (EntityID entityID : allEntityID) {
                apex = ((Area)this.worldInfo.getEntity(entityID)).getApexList();
                xInEntity = false;
                yInEntity = false;

                //同じEntity内か
                if (apex[0] > apex[2]) {
                    if(apex[0] < entry.getKey().first() && apex[2] > entry.getKey().first()) {
                        xInEntity = true;
                    }
                }else {
                    if(apex[0] > entry.getKey().first() && apex[2] < entry.getKey().first()) {
                        xInEntity = true;
                    }
                }
                if (apex[1] > apex[3]) {
                    if(apex[1] < entry.getKey().second() && apex[3] > entry.getKey().second()) {
                        yInEntity = true;
                    }
                }else {
                    if(apex[1] > entry.getKey().second() && apex[3] < entry.getKey().second()) {
                        yInEntity = true;
                    }
                }
                
                //そのEntity内にあるならば、
                if (xInEntity && yInEntity) {
                    if(resultMap.containsKey(entityID)) {
                        //resultMapにすでにKeyがあり、weightがresultMapのものよりも重い場合
                        if (resultMap.get(entityID) < entry.getValue()) {
                            resultMap.replace(entityID, resultMap.get(entityID), resultMap.get(entityID) + entry.getValue());
                        }
                        break;
                    }
                    //resultMapになければ
                    resultMap.put(entityID, entry.getValue());
                    break;
                }
            }
        }

        //weightの平均値を求める
        for (Map.Entry<EntityID, Float> entry : resultMap.entrySet()) {
            averageWeight += entry.getValue();
        }
        averageWeight = averageWeight/resultMap.size();

        //確率が最大のEntityをresultに
        float maxProbability = Float.MIN_VALUE;
        pathPlanning.setFrom(this.agentInfo.getPosition());
        while (!resultMap.isEmpty()) {
            for (Map.Entry<EntityID, Float> entry : resultMap.entrySet()) {
                if (maxProbability < entry.getValue()) {
                    maxProbability = entry.getValue();
                    result = entry.getKey();
                }
            }

            //もし、weightが平均値を下回ったらresult = null
            if (averageWeight >= maxProbability) {
                return null;
            }

            pathPlanning.setDestination(result);
            path = pathPlanning.calc().getResult();
            //Entityまで移動できるか
            if (path != null){
                if (!path.isEmpty()){
                    break;
                }
            }

            //そのEntityまで移動できなければ,対象からresultを消して
            resultMap.remove(result);
            
        }
        return result;
    }

    @Override
    public TIMRADDefaultSearch calc() {
        float probability = (messageManager.getHeardAgentHelpCount() > 0) ? 0.7f : 0.3f;
        this.multiply(makeProbabilityMap(probability, messageManager.getHeardAgentHelpCount(), this.worldInfo.getLocation(agentInfo.me())));

        if (previousNumberOfHearedHelp > -1) {
            probability = (previousNumberOfHearedHelp > 0) ? 0.7f : 0.3f;
            this.multiply(makeProbabilityMap(probability, previousNumberOfHearedHelp, this.worldInfo.getLocation(agentInfo.me())));
        }
        this.resampling();

        return this;
    }

    public void reset() {
        particleMap.clear();
        cycleOfWorking = 0;
        WORKING = false;
    }

    private Map<Pair<Integer, Integer>, Float> makeProbabilityMap(float probability, int numberOfheardHelp, Pair<Integer, Integer> agentPosition) {
        Map<Pair<Integer, Integer>, Float> probabilityMap = new HashMap<>();
        int soundReach = scenarioInfo.getRawConfig().getIntValue("comms.channels.0.range");
        int visitionReach = scenarioInfo.getPerceptionLosMaxDistance();
        for (Pair<Integer, Integer> position : this.getParticleMap().keySet()) {
            if ((position.first()-agentPosition.first())*(position.first()-agentPosition.first()) + (position.second()-agentPosition.second())*(position.second()-agentPosition.second()) <= soundReach*soundReach &&
                    !this.checkAgentLooking(this.agentInfo.getPosition(), position.first(), position.second(), visitionReach)) {
                probabilityMap.put(position, probability);
            }
            //粒子フィルタで探索が行われているということは、視界に助けを求めている市民がいないため、確率は0
            else if((position.first()-agentPosition.first())*(position.first()-agentPosition.first()) + (position.second()-agentPosition.second())*(position.second()-agentPosition.second()) > visitionReach*visitionReach) {
                probabilityMap.put(position, 0.0f);
            }
            else {
                probabilityMap.put(position, 1-probability);
            }
        }

        return probabilityMap;
    }


    /**
     * particleを管理するためのclass
     */

    public void initMap() {
        final int interval = 500;
        middlePoint = worldInfo.getLocation(agentInfo.me());
        Pair<Pair<Integer, Integer>, Pair<Integer, Integer>> worldBounds = worldInfo.getWorldBounds();
        int soundReach = scenarioInfo.getRawConfig().getIntValue("comms.channels.0.range");
        int visitionReach = scenarioInfo.getPerceptionLosMaxDistance();
        //まずある範囲に辺の半分がdの四角形を考える
        int minX = middlePoint.first() - soundReach;
        int maxX = middlePoint.first() + soundReach;
        int minY = middlePoint.second() - soundReach;
        int maxY = middlePoint.second() + soundReach;
        //四角形の頂点がマップ外にあれば、マップ内に調整
        if (minX < worldBounds.first().first()) {
            minX = worldBounds.first().first();
        }
        if (maxX > worldBounds.second().first()) {
            maxX = worldBounds.second().first();
        }
        if (minY < worldBounds.first().second()) {
            minY = worldBounds.first().second();
        }
        if (maxY > worldBounds.second().second()) {
            maxY = worldBounds.second().second();
        }
        //四角形からpositionを中心とした円を切り出し、Particleを設置
        for (int i = minX; i < maxX; i+=interval) {
            for (int j = minY; j < maxY; j+=interval) {
                if ((i-middlePoint.first())*(i-middlePoint.first())+(j-middlePoint.second())*(j-middlePoint.second()) <= soundReach*soundReach &&
                        !checkAgentLooking(agentInfo.getPosition(), visitionReach, i, j)) {
                        particleMap.put(new Pair<Integer,Integer>(i, j), 1.0f);
                }
            }
        }
    }

    public Map<Pair<Integer, Integer>, Float> getParticleMap() {
        return particleMap;
    }

    /**
     * Particle pをParticleMapに配置する
     * @param particle
     */
    public void putParticle(Pair<Integer, Integer> position) {
        if(particleMap.containsKey(position)){
            float numberOfParticle = particleMap.get(position)+1.0f;
            particleMap.replace(position, numberOfParticle);
        }
    }

    /**
     * 座標ごとに確率の掛け算を行う
     * @param probabilityMatrix
     */
    public void multiply(Map<Pair<Integer, Integer>, Float> probabilityMatrix) {
        Set<Pair<Integer, Integer>> removeList = new HashSet<>();
        for (Map.Entry<Pair<Integer, Integer>, Float> entry : particleMap.entrySet()) {
            //probabilityMatrixのKeyがparticleMap内にないならcontinue
            if (!probabilityMatrix.containsKey(entry.getKey())){
                continue;
            }
            this.particleMap.replace(entry.getKey(), (Float)entry.getValue()*probabilityMatrix.get(entry.getKey()));
            //weightが0になれば、その座標をremoveListへ
            if (entry.getValue() == 0.0f) {
                removeList.add(entry.getKey());
            }
        }
        //removeListの座標をparticleMapから削除
        for (Pair<Integer, Integer> removePosition : removeList){
            this.particleMap.remove(removePosition);
        }
    }

    /**
     * resampling
     * @return
     */
    private void resampling(){
        final int NUMBER_OF_PARTICLES = 10000;
        List<Pair<Integer, Integer>> probabilityPositionList = new ArrayList<>();
        Random random = new Random();
        for (Map.Entry<Pair<Integer, Integer>, Float> entry : particleMap.entrySet()) {
            for (int i = 0; i < entry.getValue()*100; i++){
                probabilityPositionList.add(entry.getKey());
            }
        }
        for (int i = 0; i < NUMBER_OF_PARTICLES; i++) {
            this.putParticle(probabilityPositionList.get(random.nextInt(probabilityPositionList.size())));
        }
    }

    public int size() {
        return particleMap.size();
    }

    //private void makeMap(WorldInfo this.worldInfo, AgentInfo agentInfo, Se)

    /**
     * Agentの視界にあるか確認する（視界にある = 同じEntity and 視界の範囲内）
     * @param entityID
     * @param visitionReach
     * @param x
     * @param y
     * @return
     */
    private boolean checkAgentLooking(EntityID entityID, int visitionReach, int x, int y) {
        int[] apex = ((Area)worldInfo.getEntity(entityID)).getApexList();
        boolean xInEntity = false;
        boolean yInEntity = false;
        if (x*x + y*y >= visitionReach*visitionReach) { //視界の範囲内か
            return false;
        }
        //同じEntity内か
        if (apex[0] > apex[2]) {
            if(apex[0] < x && apex[2] > x) {
                xInEntity = true;
            }
        }else {
            if(apex[0] > x && apex[2] < x) {
                xInEntity = true;
            }
        }
        if (apex[1] > apex[3]) {
            if(apex[1] < y && apex[3] > y) {
                yInEntity = true;
            }
        }else {
            if(apex[1] > y && apex[3] < y) {
                yInEntity = true;
            }
        }
        return xInEntity && yInEntity;
    }

}