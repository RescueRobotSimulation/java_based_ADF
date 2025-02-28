package TIMRAD_2025.SearchAlgorithm;
//RIOSearch_LevyDistribution

import static rescuecore2.standard.entities.StandardEntityURN.AMBULANCE_CENTRE;
import static rescuecore2.standard.entities.StandardEntityURN.AMBULANCE_TEAM;
import static rescuecore2.standard.entities.StandardEntityURN.BUILDING;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_BRIGADE;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_STATION;
import static rescuecore2.standard.entities.StandardEntityURN.GAS_STATION;
import static rescuecore2.standard.entities.StandardEntityURN.POLICE_FORCE;
import static rescuecore2.standard.entities.StandardEntityURN.POLICE_OFFICE;
import static rescuecore2.standard.entities.StandardEntityURN.REFUGE;
import adf.core.agent.action.common.ActionMove;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.StandardCommunicationModule;
import adf.core.component.communication.CommunicationMessage;
import adf.core.agent.communication.standard.bundle.MessageUtil;
import adf.core.agent.communication.standard.bundle.StandardMessage;
import adf.core.agent.communication.standard.bundle.centralized.CommandFire;
import adf.core.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.core.agent.communication.standard.bundle.information.MessageAmbulanceTeam;
import adf.core.agent.communication.standard.bundle.information.MessageFireBrigade;
import adf.core.agent.communication.standard.bundle.information.MessagePoliceForce;
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
import adf.impl.tactics.utils.MessageTool;
import rescuecore.commands.Command;
// import firesimulator.world.AmbulanceTeam;
// import firesimulator.world.Civilian;
import rescuecore.commands.Update;

import org.apache.log4j.Logger;

import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;
import adf.core.agent.communication.standard.bundle.StandardMessagePriority;

import java.util.*;
import java.lang.Math;
import java.awt.geom.Rectangle2D;

// import javax.measure.quantity.Area;
// import javax.swing.text.html.parser.Entity;

/**
  * @author Mohsen
  * @since  2025.2.25
  * @brief  
  */

public class RIOSearchLevyDistribution extends TIMRADDefaultSearch {

  private   EntityID result         = null;
  private   float    moveDistance   = 0;
  private   Random   random         = new Random();
  private   float    moveDirection  = 0; 
  private   int      remuneration   = 100;

  private   Collection<EntityID> unsearchedBuildingIDs = new ArrayList<>();
  private   List<EntityID>       previousPathList      = new ArrayList<>();
  private   List<EntityID>       path            = new ArrayList<>();  

  @Override
  public TIMRADDefaultSearch updateInfo(MessageManager messageManager) {
    super.updateInfo(messageManager);
    if (this.result != null){
      if (this.agentInfo.me() instanceof AmbulanceTeam at) {
        messageManager.addMessage(new MessageAmbulanceTeam(true, at, MessageAmbulanceTeam.ACTION_MOVE, result));
      }
      else if (this.agentInfo.me() instanceof FireBrigade fb) {
        messageManager.addMessage(new MessageFireBrigade(true, fb, MessageFireBrigade.ACTION_MOVE, result));
      }
      else if (this.agentInfo.me() instanceof PoliceForce pf) {
        messageManager.addMessage(new MessagePoliceForce(true, pf, MessagePoliceForce.ACTION_MOVE, result));
      }
    }
    
  //   for (CommunicationMessage message : messageManager.getReceivedMessageList()){
  //     MessageUtil.reflectMessage(this.worldInfo, (StandardMessage)message);
  // }

    //previousPath
    for (int i = 1; i < this.agentInfo.getTime(); i++){
      this.previousPathList.add(this.worldInfo.getPosition(i, (Human)this.agentInfo.me()).getID());
    }

    if (messageManager.getHeardAgentHelpCount() == 0) {
      this.remuneration -= 1;
    }
    else {
      this.remuneration = 100;
    }

    if (remuneration < 10) {
      this.remuneration = 10;
    }

    return this;
  }


  @Override
  public TIMRADDefaultSearch calc() {
    for (CommunicationMessage cm : messageManager.getReceivedMessageList()) {
      MessageUtil.reflectMessage(worldInfo, (StandardMessage)cm);
    }
    // if (this.agentInfo.getExecutedAction(-1) instanceof ActionMove am) {
    //   if (this.isStopping(this.worldInfo.getLocation(this.agentInfo.me().getID()), this.worldInfo.getLocation(-1, this.agentInfo.me().getID()))) {
    //     //this.result = am.getPath().get((am.getPath()).size()-1);
    //     return this;
    //   }
    //   // if (this.agentInfo.getPosition().getValue() != am.getPath().get((am.getPath()).size()-1).getValue()) {
    //   //   return this;
    //   // }
    // }

    Set<EntityID>  distinationsSet = new HashSet<>();
    result = null;

    //PF and 市民を背負っているATは建物を探索する必要はない。
    if ((this.agentInfo.me() instanceof AmbulanceTeam at && this.agentInfo.someoneOnBoard() != null)) {
      for (int t = -20; t <= 0; t++) {
        distinationsSet.addAll(this.worldInfo.getEntityIDsOfType(t, StandardEntityURN.ROAD));
      }
      //探索範囲を拡大するためNeighborを追加
      for (EntityID entityID : this.worldInfo.getEntityIDsOfType(StandardEntityURN.ROAD)) {
        for (EntityID neighbourID : ((Area)this.worldInfo.getEntity(entityID)).getNeighbours()){
          if (this.worldInfo.getEntity(neighbourID) instanceof Road){
            distinationsSet.add(neighbourID);
          }
        }
      }
    }
    else {
      for (int t = -20; t <= 0; t++) {
        distinationsSet.addAll(this.worldInfo.getEntityIDsOfType(t, StandardEntityURN.BUILDING, StandardEntityURN.ROAD));
      }
      //探索範囲を拡大するためNeighborを追加
      for (EntityID entityID : this.worldInfo.getEntityIDsOfType(StandardEntityURN.BUILDING, StandardEntityURN.ROAD)) {
        for (EntityID neighbourID : ((Area)this.worldInfo.getEntity(entityID)).getNeighbours()){
          if (this.worldInfo.getEntity(neighbourID) instanceof Building){
            distinationsSet.add(neighbourID);
          }
        }
        //distinationsSet.addAll(((Area)this.worldInfo.getEntity(entityID)).getNeighbours());
      }
    }
    //探索の目的地から、役割のある建物(避難所、消防署など)を削除
    distinationsSet.removeAll(this.worldInfo.getEntityIDsOfType(REFUGE,FIRE_STATION,POLICE_OFFICE,AMBULANCE_CENTRE));
    distinationsSet.removeAll(previousPathList);

    // //建物の入り口にある小さなRoadを削除
    // List<StandardEntity>distinateBuildingList = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.BUILDING)); 
    // int min = Integer.MAX_VALUE;
    // EntityID smallestRoad = null;
    // for (StandardEntity building : distinateBuildingList){
    //   //家に隣接するの最小の道路(玄関)を探す
    //   for (EntityID roadID : ((Building)building).getNeighbours()){
    //     int[] apex = ((Area)this.worldInfo.getEntity(roadID)).getApexList();
    //     if (min > Math.abs((apex[0] - apex[2])*(apex[1] - apex[3]))){
    //       min = Math.abs((apex[0] - apex[2])*(apex[1] - apex[3]));
    //       smallestRoad = roadID;
    //     }
    //   }
    //   //玄関を削除
    //   if (distinationsSet.contains(smallestRoad)){
    //     distinationsSet.remove(smallestRoad);
    //   }
    // }

    //moveDistanceが0になったとき
    if(this.moveDistance <= 0 || this.isStopping()){
      Rectangle2D worldBounds = this.worldInfo.getBounds();
      //moveDistanceの最大値を世界の斜めの半分に
      float minWorldBound = worldBounds.getHeight() > worldBounds.getWidth() ? (float)worldBounds.getWidth() : (float)worldBounds.getHeight();
      this.moveDistance = LevyDistribution(this.random.nextFloat(this.remuneration), 0, 5)*minWorldBound;

      //ルーレット選択でresultを決定
      pathPlanning.setFrom(this.agentInfo.getPosition());
      this.moveDirection = this.rouletteSelct(distinationsSet).get(random.nextInt(16));
      //this.moveDirection = this.maximunSelect(distinationsSet);
      reset();
    }
    //moveDirectionが決定しているなら、その方向に最も近いEntityに移動
    this.result = this.determinDistination(distinationsSet, moveDirection);

    if (!(this.agentInfo.me() instanceof PoliceForce)) {
      for (EntityID neighbourID : ((Area)this.worldInfo.getEntity(result)).getNeighbours()) { 
        if (this.worldInfo.getEntity(neighbourID) instanceof Building) {
          this.result = neighbourID;
        }
      }
    }
    
    if (this.result != null) {
      moveDistance -= calcMovedDistance(this.pathPlanning.getResult(this.agentInfo.getPosition(), this.result));
      return this;
    }

    //これより下が実行されるということは探索先が見つかっていないと言うこと
    //pathがnullなら、考えられるすべてのEntityで探索先を探す
    distinationsSet = new HashSet<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.ROAD, StandardEntityURN.BUILDING, StandardEntityURN.REFUGE, StandardEntityURN.FIRE_STATION, StandardEntityURN.GAS_STATION));
    //Neighborを追加
    for (EntityID entityID : this.worldInfo.getEntityIDsOfType(StandardEntityURN.ROAD, StandardEntityURN.BUILDING, StandardEntityURN.REFUGE, StandardEntityURN.FIRE_STATION, StandardEntityURN.GAS_STATION)) {
      distinationsSet.addAll(((Area)this.worldInfo.getEntity(entityID)).getNeighbours());
    }

    distinationsSet.remove(this.agentInfo.getPosition());
    this.result = this.determinDistination(distinationsSet, moveDirection);

    if (this.result != null) {
      moveDistance -= this.calcMovedDistance(this.pathPlanning.getResult(this.agentInfo.getPosition(), this.result));
      return this;
    }

    System.out.println("No");

    return this;
  }

  private void reset() {
    this.unsearchedBuildingIDs.clear();
    EntityID nearestBuilding = null;
    int distanceNearestBuilding = Integer.MAX_VALUE;
    for (StandardEntity buildingID : this.worldInfo.getEntitiesOfType(StandardEntityURN.BUILDING)){
      if (distanceNearestBuilding > this.worldInfo.getDistance(buildingID, this.agentInfo.me())){
        distanceNearestBuilding = this.worldInfo.getDistance(buildingID, this.agentInfo.me());
        nearestBuilding = buildingID.getID();
      }
    }
    int myPositionClusterIndex = clustering.getClusterIndex(nearestBuilding);
    List<StandardEntity> clusterEntities = new ArrayList<>(this.clustering.getClusterEntities(myPositionClusterIndex));
    if (clusterEntities != null && clusterEntities.size() > 0) {
      for (StandardEntity entity : clusterEntities) {
        if (entity instanceof Building && entity.getStandardURN() != REFUGE) {
          this.unsearchedBuildingIDs.add(entity.getID());
        }
      }
    }
    else {
      this.unsearchedBuildingIDs.addAll(this.worldInfo.getEntityIDsOfType(BUILDING));
    }
  }

  public EntityID getTarget() {
    return this.result;
  }

  /**レビィ分布を計算して正規化*/
  private float LevyDistribution(float x, int mu, int c){
    if(x == mu){
        return 0; //発散するので０に
    }
    //最大値を求め、最大値を1に
    //0.46は0.001で離散化した場合のLevy分布の大体の最大値
    return (float)((Math.sqrt(c/(2*3.14))*Math.exp(-c/(2*(x-mu)))/Math.pow((x-mu),3/2))/0.46f);
  }

  private float multivariateNormalDistribution(float x, float y, float distributeX, float distributeY) {
    return (float)(1/(Math.sqrt(2*3.14*distributeX))*Math.exp(Math.pow(x, 2)/2 + Math.pow(y, 2)/2));
  }

  /**移動先の決定 */
  private EntityID determinDistination(Collection<EntityID> areas, float directionOfLevyWalk) {
    Set<EntityID> distinations = new HashSet<>(areas);
    EntityID direction = null;
    float directionDiff = Float.MAX_VALUE;
    this.pathPlanning.setFrom(this.agentInfo.getPosition());
    directionDiff = Float.MAX_VALUE;
    while (!distinations.isEmpty()) {
      directionDiff = Float.MAX_VALUE;
      //角度の差が最小のEntityIDを探す
      for (EntityID entityID : distinations) {
        if (directionDiff > Math.abs(this.calcAngle(this.agentInfo.me().getID(), entityID) - directionOfLevyWalk)) {
          direction = entityID;
          directionDiff = Math.abs(this.calcAngle(this.agentInfo.me().getID(), entityID) - directionOfLevyWalk) ;
        }
      }
      if (directionDiff > 3.14/2) {
        directionOfLevyWalk = this.rouletteSelct(distinations).get(random.nextInt(16));
      }

      //そこまで行けるなら目的地に
      this.pathPlanning.setDestination(direction);
      this.path = this.pathPlanning.calc().getResult();
      if (path != null) { 
        if (!path.isEmpty()) {
          break;
        }
      }

      //行けないなら目的地のSetから削除して、再検索
      distinations.remove(direction);
    }
    //すべてのdistinationsで移動ができなかった
    return direction;
  }

  //交差点か判別
  private Set<EntityID> getRoadsByArea(Collection<EntityID> roadsWithoutEntrance, EntityID areaID){

    Area area = (Area)this.worldInfo.getEntity(areaID);
    Set<EntityID>  neighbourRoads = new HashSet<>(); //交差点に隣接する道。
    List<EntityID> neighbours     = area.getNeighbours(); //隣接するEntityを取得

    for(EntityID entityID : neighbours){
      if(!roadsWithoutEntrance.contains(entityID)){ //玄関を含むと交差点の扱いになるので、玄関を削除
        continue;
      }
      if(StandardEntityURN.ROAD == this.worldInfo.getEntity(entityID).getStandardURN()){
        neighbourRoads.add(entityID);
      }
    }
      return neighbourRoads;
  }

  /**ルーレット選択*/
  private List<Float> rouletteSelct(Collection<EntityID> roads){
    float  theta = 0;

    Map<Float, Integer> probabilityOfMoveDirection = sumProbabilityOfMoveDirection(this.agentInfo.me().getID());
    List<Float> thetaList = new ArrayList<>();
    List<Float> probabilityList = new ArrayList<>(); //この中から確率的に１つIDを選ぶ
    // 　　　　　　probabilityList = {a, a, b} --- aの確率 2/3, bの確率 1/3


    //thetaListの作成
      for (float i = 0; i < 2*3.14f; i+=3.14f/8) {
        thetaList.add(i);
      }

    // int maxProbability = Collections.max(probabilityOfMoveDirection.values());
    // for (Map.Entry<Float, Integer> entry : probabilityOfMoveDirection.entrySet()) {
    //   if (entry.getValue() == maxProbability) {
    //     thetaList.add(entry.getKey());
    //   }
    //   if (entry.getValue() == minProbability) {
    //     thetaList
    //   }
    // }
    


    //すべての方向が必ず選ばれるように
    probabilityList.addAll(thetaList);

    //確率配列を作成, 確率密度関数のリーマン和を確率とする。
    float sumAngleProbability = 0;
    for (float angle : thetaList){
      sumAngleProbability = 0;
      //リーマン和(幅: 0.1)
      for (float i = angle - (float)(thetaList.get(1) - thetaList.get(0))/2; i < angle + (float)(thetaList.get(1) - thetaList.get(0))/2; i += 0.5f) {
        sumAngleProbability += calcLagrange(probabilityOfMoveDirection, i)*i;
      }
      for (int i = 0 ; i < sumAngleProbability*2; i++){
        probabilityList.add(angle);
      }
    }
    //確率配列からランダムに選択
    return probabilityList;
  }

  /**内積と外積から角度を計算 */
  private float calcAngle(EntityID start, EntityID distination) {
    float angle = 0;
    //cross product　外積から角度を計算
    //double norm = Math.sqrt(Math.pow(this.worldInfo.getLocation(distination).first(), 2) + Math.pow(this.worldInfo.getLocation(distination).second(), 2)) * Math.sqrt(Math.pow(this.worldInfo.getLocation(start).first(), 2) + Math.pow(this.worldInfo.getLocation(start).second(), 2));
    double norm = Math.sqrt(Math.pow(this.worldInfo.getLocation(distination).first()-this.worldInfo.getLocation(start).first(), 2) + Math.pow(this.worldInfo.getLocation(distination).second()-this.worldInfo.getLocation(start).first(), 2));
    //angle = (float)Math.acos((this.worldInfo.getLocation(distination).first()*this.worldInfo.getLocation(start).first() + this.worldInfo.getLocation(distination).second()*this.worldInfo.getLocation(start).second())/norm);
    angle = (float)Math.acos((this.worldInfo.getLocation(distination).first()-this.worldInfo.getLocation(start).first())/norm);
    //inner product　内積が0以下なら+pi
    // if ((float)Math.asin(((this.worldInfo.getLocation(distination).first()*this.worldInfo.getLocation(start).first() - this.worldInfo.getLocation(distination).second()*this.worldInfo.getLocation(start).second())/norm)) < 0) {
    //   angle += 3.14f;
    // }
    if ((float)Math.asin((this.worldInfo.getLocation(distination).second()-this.worldInfo.getLocation(start).second())/norm) < 0) {
      angle += 3.14f;
    }
    return angle;
  }

  /**Map<theta, probability>配列を標準化 */
  private Map<Float, Integer> normalizeValueOfMap(Map<Float, Float> list){
    Map<Float, Integer> normalizedList = new HashMap<>();
    if (list.size() == 1){
      List<Float> keyList = new ArrayList<>(list.keySet());
      normalizedList.put(keyList.get(0), 1);
      return normalizedList;
    }

    float max = Float.MIN_VALUE;
    float min = Float.MAX_VALUE;

    //最大値と最小値を求める
    for (float x : list.values()) {
      if (x > max){
        max = x;
      }
      if (x < min) {
        min = x;
      }
    }

    //標準化
    for (Map.Entry<Float, Float> entry : list.entrySet()){
      //少数第3位までを確率に反映
      normalizedList.put(entry.getKey(), (int)(1000*(entry.getValue()-min)/(max-min)));
    }

    return normalizedList;
  }

  /**移動距離を計算*/
  private int calcMovedDistance(List<EntityID> travelPath){
    int movedDistance = 0;
    if (travelPath == null) {
      return 0;
    }
    int[] apexList;
    for (EntityID entityID : travelPath){
      apexList = ((Area)this.worldInfo.getEntity(entityID)).getApexList();
      movedDistance += (int)Math.sqrt((Math.pow(apexList[1]-apexList[3],2) + Math.pow(apexList[2]-apexList[4],2))) + 0.5f;
    }

    return movedDistance;
  } 
  /**移動方向の確率の総和を計算*/
  public Map<Float, Integer> sumProbabilityOfMoveDirection(EntityID fromAgentID) {
    float angle = 0;
    float directionProbability = 0;

    Map<Float, Integer> probabilityDirectionList = new TreeMap<>(); //theta, priority
    Map<Float, Float>   repaireCostList          = new TreeMap<>();
    List<EntityID> targetOfIndicator             = new ArrayList<>();

    //自分と同じのエージェントがいる方向の選ばれる確率を減少
    for (Map.Entry<Float, Integer> entry : calcProbability(this.worldInfo.getEntityIDsOfType((this.agentInfo.me()).getStandardURN()), -1).entrySet()) {
      probabilityDirectionList.put(entry.getKey(), entry.getValue());
    }
    //自分と違う種類のエージェントが選ばれる確率を減少
    for (EntityID agentID : this.worldInfo.getEntityIDsOfType(StandardEntityURN.POLICE_FORCE, StandardEntityURN.AMBULANCE_TEAM, StandardEntityURN.FIRE_BRIGADE)){
      if (this.worldInfo.getEntity(agentID).getStandardURN().getURNId() != this.agentInfo.me().getStandardURN().getURNId()){
        targetOfIndicator.add(agentID);
      }
    }
    for (Map.Entry<Float, Integer> entry : calcProbability(targetOfIndicator, 0.3f).entrySet()) {
      probabilityDirectionList.put(entry.getKey(), entry.getValue());
    }
    //避難所にいないcivilianによって方向が選ばれる確率を増加
    targetOfIndicator = new ArrayList<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.CIVILIAN));
    for (EntityID civilianID : this.worldInfo.getEntityIDsOfType((this.agentInfo.me()).getStandardURN())) {
      if (this.worldInfo.getPosition(civilianID).getStandardURN().getURNId() == StandardEntityURN.REFUGE.getURNId()){
        targetOfIndicator.remove(civilianID);
      }
    }
    if (!targetOfIndicator.isEmpty()){
      for (Map.Entry<Float, Integer> entry : calcProbability(targetOfIndicator, 1).entrySet()) {
        probabilityDirectionList.put(entry.getKey(), entry.getValue());
      }
    }

    // //前のサイクルで通った道の選ばれる確率を下げる。
    // //現在の場所まで参照してしまうと、Roadの面積が大きかった場合に反復動作が発生するためremove
    // this.previousPathList.remove(this.worldInfo.getPosition((this.agentInfo.me()).getID()).getID());
    // for (Map.Entry<Float, Integer> entry : calcProbability(this.previousPathList, -1).entrySet()) {
    //   probabilityDirectionList.put(entry.getKey(), entry.getValue());
    // }

    //Commandを取得し、命令が出されているAgentの位置によって確率を決める
    targetOfIndicator.clear();
    // for (CommunicationMessage message : messageManager.getReceivedMessageList()) {
    //   if (this.agentInfo.me() instanceof AmbulanceTeam && message instanceof MessageAmbulanceTeam ma) {
    //     targetOfIndicator.add(ma.getTargetID());
    //   }
    //   else if (this.agentInfo.me() instanceof FireBrigade && message instanceof MessageFireBrigade mf) {
    //     targetOfIndicator.add(mf.getTargetID());
    //   }
    //   else if (this.agentInfo.me() instanceof PoliceForce && message instanceof MessagePoliceForce mp) {
    //     targetOfIndicator.add(mp.getTargetID());
    //   }
    // }
    // for (Map.Entry<Float, Integer> entry : calcProbability(targetOfIndicator, 1).entrySet()) {
    //   probabilityDirectionList.put(entry.getKey(), entry.getValue());
    // }

    //POLICE_FORCEなら、瓦礫の量によって確率を決める。
    if (this.agentInfo.me().getStandardURN() == StandardEntityURN.POLICE_FORCE){
      repaireCostList.clear();
      for (EntityID entityID : this.worldInfo.getEntityIDsOfType(StandardEntityURN.ROAD)){
        angle = calcAngle(fromAgentID, entityID);
        angle = (float)((int)(angle*10)/10);
        if (!((Road)this.worldInfo.getEntity(entityID)).isBlockadesDefined()) {
          continue;
        }
        for (EntityID blockadesID : ((Area)this.worldInfo.getEntity(entityID)).getBlockades()) {
          if (((Blockade)this.worldInfo.getEntity(blockadesID)).isRepairCostDefined() == false){
            continue;
          }
          if (repaireCostList.containsKey(angle)) {
            directionProbability = (float)(repaireCostList.get(angle) + ((Blockade)this.worldInfo.getEntity(blockadesID)).getRepairCost());
            repaireCostList.put(angle, directionProbability);
          }
          else {
            repaireCostList.put(angle, (float)((Blockade)this.worldInfo.getEntity(blockadesID)).getRepairCost());
          }
        }
      }
      if(!repaireCostList.isEmpty() || repaireCostList.size() != 0){
        for (Map.Entry<Float, Integer> entry : normalizeValueOfMap(repaireCostList).entrySet()) {
          probabilityDirectionList.put(entry.getKey(), entry.getValue());
        }
      }
    }
    //円を作るために[thetaの最小値+2π]にthetaの最小値の生起確率をいれる。
    //List<Float> thetaList = new ArrayList<>(probabilityDirectionList.keySet());
    float minTheta = Collections.min(probabilityDirectionList.keySet());
    probabilityDirectionList.put(minTheta+2*3.14f, probabilityDirectionList.get(minTheta));

    return probabilityDirectionList;
  }

  /**方向を選択する確率を計算 
   * entityIDSetから角度ごとに確率をvalueごとに増加*/
  private Map<Float, Integer> calcProbability(Collection<EntityID> entityIDSet, float value){

    EntityID fromPoint = this.agentInfo.me().getID();
    float    theta = 0;
    float    probability = 0;

    Map<Float, Integer> probabilityDirectionList           = new TreeMap<>(); //theta, priority
    Map<Float, Float> noInitializeProbabilityDirectionList = new TreeMap<>();

    if(entityIDSet.isEmpty() || fromPoint == null){
      return probabilityDirectionList;
    }

    //計算
    for (EntityID entityID : entityIDSet){
      if (this.worldInfo.getLocation(entityID) == null) {
        continue;
      }
      theta = calcAngle(fromPoint, entityID);
      theta = (float)((int)(theta*10)/10); //少数第２位を切り捨て
      probability += value/(1 + Math.abs(this.moveDistance-this.worldInfo.getDistance(entityID, fromPoint)));
      
      if (probabilityDirectionList.containsKey(theta)) {
        probability = noInitializeProbabilityDirectionList.get(theta) + value/(1 + Math.abs(this.moveDistance-this.worldInfo.getDistance(entityID, fromPoint)));
        noInitializeProbabilityDirectionList.put(theta, probability);
      }
      else {
        noInitializeProbabilityDirectionList.put(theta, probability);
      }
    }
    //正規化
    for (Map.Entry<Float, Integer> entry : normalizeValueOfMap(noInitializeProbabilityDirectionList).entrySet()) {
      probabilityDirectionList.put(entry.getKey(), entry.getValue());
    }

    return probabilityDirectionList;
  }

  /**ラグランジュ補完*/
  private int calcLagrange(Map<Float, Integer> directionProbabirity, float x) {
    float ans = 0;
    for (float theta1 : directionProbabirity.keySet()){
      float z = 1;
      for (float theta2 : directionProbabirity.keySet()){
        if (theta1 != theta2){
          z = z*(x - theta2)/(theta1 - theta2);
        }
      }
      ans += z*directionProbabirity.get(theta1);
    }
    return (int)ans;
  }

  /**最も遠いareaID内の座標を求める */
  private Pair<Integer, Integer> calcFarthestPoint(EntityID areaID, EntityID agentID){
    int[] areaPoint = ((Area)this.worldInfo.getEntity(areaID)).getApexList();
    Pair<Integer, Integer> agentPoint = this.worldInfo.getLocation(agentID);

    int farthestX = Math.max(Math.abs(areaPoint[0] - agentPoint.first()), Math.abs(areaPoint[2] - agentPoint.first()));
    int farthestY = Math.max(Math.abs(areaPoint[0] - agentPoint.second()), Math.abs(areaPoint[2] - agentPoint.second()));

    Pair<Integer, Integer> farthestPoint = new Pair<>(farthestX, farthestY);

    return farthestPoint;
  }

  private boolean isStopping() {
    Pair<Integer,Integer> location = this.worldInfo.getLocation(this.agentInfo.me().getID());
    Pair<Integer, Integer> previousLocation = this.worldInfo.getLocation(-1, this.agentInfo.me().getID());
    boolean isStopping = false;
    int threadHoldReach = 20;
    if (!(this.agentInfo.getExecutedAction(-1) instanceof ActionMove)) {
      return isStopping;
    }
    if (Math.pow(location.first() - previousLocation.first(), 2) + Math.pow(location.second() - previousLocation.second(), 2) < Math.pow(threadHoldReach, 2)) {
      if (!(((this.agentInfo.getPositionArea()).getBlockades()) == null)){
        CommandPolice commandRemove = new CommandPolice(true ,StandardMessagePriority.NORMAL, this.agentInfo.me().getID(), this.agentInfo.getPosition(), CommandPolice.ACTION_CLEAR);
        messageManager.addMessage(commandRemove);
      }
      isStopping = true;

    }

    return isStopping;
  }

  private float maximunSelect(Collection<EntityID> roads) {
    float  theta = 0;
    Map<Float, Integer> probabilityOfMoveDirection = sumProbabilityOfMoveDirection(this.agentInfo.me().getID());
    List<Float> thetaList = new ArrayList<>();
    List<Float> probabilityList = new ArrayList<>(); //この中から確率的に１つIDを選ぶ
    float result = 0;
    // 　　　　　　probabilityList = {a, a, b} --- aの確率 2/3, bの確率 1/3

    //thetaListの作成
      for (float i = 0; i < 2*3.14f; i+= 2*3.14f/16) {
        thetaList.add(i);
      }

      //すべての方向が必ず選ばれるように
      probabilityList.addAll(thetaList);

      if (probabilityOfMoveDirection == null) {
        return probabilityList.get(random.nextInt(16));
      }

      //確率配列を作成, 確率密度関数のリーマン和を確率とする。
      float sumAngleProbability = 0;
      float maxAngleProbability = Float.MIN_VALUE;
      for (float angle : thetaList){
        sumAngleProbability = 0;
        //リーマン和(幅: 0.1)
        for (float i = angle - (float)(thetaList.get(1) - thetaList.get(0))/2; i < angle + (float)(thetaList.get(1) - thetaList.get(0))/2; i += 0.5f) {
          sumAngleProbability += calcLagrange(probabilityOfMoveDirection, i)*i;
        }

        if (sumAngleProbability > maxAngleProbability) {
          maxAngleProbability = sumAngleProbability;
          result = angle;
        }
      }
    return result;
  }
}