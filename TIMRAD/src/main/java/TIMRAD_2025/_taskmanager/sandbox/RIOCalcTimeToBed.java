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
 * @author Nomoto(17th), Ikegami(18th), Kato(18th)
 * @since 2022
 * @brief 市民やエージェントが受けるダメージを計算
 */
public class RIOCalcTimeToBed extends RIOEDefault {


    /** 待ち時間を格納する変数 **/
    private Map<EntityID, Double> waitingTimeMap    = new HashMap<>();
    /**各避難所のマンハッタン距離**/
    private Map<EntityID,Integer> ManhattanDistancetoRefuge = new HashMap<>();

    /**優先度
     *  Map<市民のID, Map<避難所のID, 市民から見た避難所の優先度>>
    */
    private Map<EntityID, Double> priorityList   = new HashMap<>();//各避難所のPriority(updateで使用)
    private double bestPriority  = 0;
    private double worstPriority = 1.7976931348623157E308;  //大きい値にすれば最小値の条件分岐が簡単に

    //マップ内にある全ての情報を取得
    private Collection<EntityID>  allRefugeIDs   = new HashSet<>();//避難所の情報取得
    private Collection<EntityID>  allCivilianIDs = new HashSet<>(); //市民の情報取得

    // WaitingTime から
    private Map<EntityID, Integer>      numberOfBeds           = new HashMap<>();// 各避難所のべットの数
    private Map<EntityID, Integer>      eachRefugeQueueSize    = new HashMap<>();// 各避難所の行列の長さを取得
    private Map<EntityID, Integer>      eachRefugeOccupiedBeds = new HashMap<>();// 各避難所のべットの使用数を取得
    private Map<EntityID,List<Integer>> noDamageCivilianList   = new HashMap<>();//(refugeのID,(人数))
    private Map<EntityID,List<Integer>> allCiviliansList       = new HashMap<>();//(refugeのID,(時間経過後の人数))
    private Map<EntityID,List<Integer>> confirmationTime       = new HashMap<>();//agentがRefugeの状況を確認した時間
    private double                      allCivilianVariations  = 0;//getCrowdで取得できます

    private MessageManager              messageManager         = new MessageManager();

    //ManhattanDistanceから
    private double clearDistance;
    private Map<EntityID, Integer> TimeOfManhattanDistance = new HashMap<>();
    private Human me;

    @Override
    protected IRIOEvaluator init(){

        receiveMessageCivilian(messageManager); //市民の情報を受信(worldInfoに入ります)
        receiveMessageRefuge(messageManager);   //Refugeの情報を受信

        this.allCivilianIDs = new HashSet<>(worldInfo.getEntityIDsOfType(CIVILIAN));

        return this;
    }

    /// ---- 正規化関数 ----
    /// summary : 最も良い値 = 0
    @Override
    protected double bestPriority() {
        return this.bestPriority; //ダメージをほぼ受けていない
    }

    /// summary : 最も悪い値 = 最も遠いキョリ * 建物の数 * 優先しない場合の重み
    @Override
    protected double worstPriority() {
        return this.worstPriority;
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
    /// summary : 各サイクルごとの事前処理, buildingsを更新
    @Override
    public IRIOEvaluator update(){

        receiveMessageCivilian(messageManager); //市民の情報を受信(worldInfoに入ります)
        receiveMessageRefuge(messageManager);   //Refugeの情報を受信

        //Map<EntityID, Integer> ManhattanDistance = new HashMap<>();
        this.allRefugeIDs  = this.worldInfo.getEntityIDsOfType(REFUGE);
        this.me = (Human)agentInfo.me();
        //この辺にWaitingTimeListとManhattanDistance追記
        for(EntityID refugeID : allRefugeIDs){
            //ManhattanDistance.put(refugeID, this.getManhattanDistance(refugeID, me));
            this.ManhattanDistancetoRefuge.put(refugeID, this.getManhattanDistance(refugeID, me));
            this.waitingTimeMap.put(refugeID,this.calcWaitingTimeLittle(refugeID));
        }

        //以下待ち行列
        //道順」を取得してリストに加える

        List<EntityID> path = pathPlanning.calc().getResult();

        Set<EntityID>          entityIDs                   = this.worldInfo.getChanged().getChangedEntities();
        Map<EntityID, Integer> eachRefugeNoDamageCivilians = new HashMap<>(); // ダメージを受けていない市民の人数
        Map<EntityID, Integer> eachRefugeAllCivilians      = new HashMap<>(); // 各避難所の市民の数
        Map<EntityID, Integer> eachRefugeConfirmationTime  = new HashMap<>();
        int                    Time; //観測した時間

        //周囲に避難所がある場合、避難所の情報を取得する
        for(EntityID entityID : entityIDs){
            StandardEntity entity       = worldInfo.getEntity(entityID);
            StandardEntityURN entityURN = entity.getStandardURN();

            //Agentの周りに避難所があるとき
            if(entityURN != REFUGE){
                continue;    //entityが避難所ではないので以降の処理を飛ばす
            }

            sendMessageRefuge(entityID, messageManager);

            Refuge refuge = (Refuge)entity;
            Time = agentInfo.getTime(); //Refugeを観測した時間を取得

            if(eachRefugeConfirmationTime.containsKey(entityID) == false){
                eachRefugeConfirmationTime.put(entityID, Time);
            }
            else{
                eachRefugeConfirmationTime.replace(entityID, Time);
            }
            eachRefugeAllCivilians      = getCivilianInfoAtEachRefuge(entityID, "AllCivilians");
            eachRefugeNoDamageCivilians = getCivilianInfoAtEachRefuge(entityID, "noDamageCivilians");

            //避難所のIDがBedNumberかrefugeQueueSizeのキーとして、登録されていないとき
            if(numberOfBeds.containsKey(entityID) == false || eachRefugeQueueSize.containsKey(entityID) == false){
                //キーと情報を取得する
                numberOfBeds.put(entityID, refuge.getBedCapacity());
                eachRefugeQueueSize.put(entityID, refuge.getWaitingListSize());
                eachRefugeOccupiedBeds.put(entityID, refuge.getOccupiedBeds());
            }
            else{
                //キーの情報を更新する
                eachRefugeQueueSize.replace(entityID, refuge.getWaitingListSize());
                eachRefugeOccupiedBeds.replace(entityID, refuge.getOccupiedBeds());
            }
        }


        for(EntityID refugeID : allRefugeIDs){
            //上部で得た情報を配列に加工し、MapでRefugeIDで扱えるようにする
            this.allCiviliansList     = this.packageData(eachRefugeAllCivilians, refugeID);
            this.noDamageCivilianList = this.packageData(eachRefugeNoDamageCivilians, refugeID);
            this.confirmationTime     = this.packageData(eachRefugeConfirmationTime , refugeID);

            //待ち時間の取得
            this.waitingTimeMap.put(refugeID, calcWaitingTimeLittle(refugeID));

            this.priorityList = this.calcPriority(refugeID, (Human)agentInfo.me());
        }

        this.allCivilianIDs = this.worldInfo.getEntityIDsOfType(StandardEntityURN.CIVILIAN);
        this.me = (Human)agentInfo.me();
        //以下マンハッタン距離(時間)
        for(EntityID refugeID : allRefugeIDs){
            this.TimeOfManhattanDistance.put(refugeID,getManhattanDistance(refugeID, me));
        }

        return this;
    }

    /// summary : 優先度計算(編集中), 不安定度計算(ココでは何もしない)
    @Override
    public RIODynamicTaskKey calc(EntityID evaluateeID) {
        double priority     = 0;
        double unstableness = 1;
        if(worldInfo.getEntity(evaluateeID).getStandardURN() != REFUGE || this.agentInfo.someoneOnBoard() == null){
            return new RIODynamicTaskKey(evaluateeID, priority, unstableness);
        }

        priority = this.priorityList.get(evaluateeID);
        return new RIODynamicTaskKey(evaluateeID, priority, unstableness);
    }


    /** 市民がrefugeに入るまでの時間*
     * @param
     * @return double 市民が治療を受けるまでの時間
    */
    private double getTimeUntilBed(EntityID refugeID, EntityID civilianID) {

        /// 本処理

        //↓待ち行列とマンハッタン距離を追加したあとは0にしておいてください。
        double timeUntilBed= 0;//　動かすための仮の優先度

        timeUntilBed = waitingTimeMap.get(refugeID) + (this.ManhattanDistancetoRefuge.get(refugeID));

        return timeUntilBed;
    }

    /**ダメージを計算*
     * @param
     * @return double 市民が受けるダメージ
    */
    private double calcDamage(double k, double l, double observationDamage, double t) {
        double damage = 0; //ダメージ
        damage = -(1/k)*Math.log(Math.cos(Math.sqrt(k*l))*t-Math.sqrt(k/l)*observationDamage*Math.sin(Math.sqrt(k*l))*t) ;

        return damage;
    }

    /**優先度を取得*
     * @param
     * @return double 優先度の計算
    */
    private Map<EntityID, Double> calcPriority(EntityID refugeID, Human me){ //Map<市民のID, Map<避難所のID, 市民から見た避難所の優先度>>
        double priority = 0; //優先度
        double damageBury = 0; //埋没ダメージ
        double damageCollapse = 0; //崩壊ダメージ
        double damageFire = 0; //火災ダメージ

        Map<EntityID, Double> priorityList  = new HashMap<>(); //<RefugeID, 避難所の優先度>
        if(this.agentInfo.someoneOnBoard() == null){
            priorityList.put(refugeID, 0.0);
            return priorityList;
        }
        EntityID civilianID     = agentInfo.someoneOnBoard().getID();
        double   observedDamage = ((Human)this.worldInfo.getEntity(civilianID)).getDamage(); //観測時ダメージ

        double   timeuntilbed = this.getTimeUntilBed(refugeID, civilianID); //時間
        double   totalDamage  = 0; //合計ダメージ

        damageBury = calcDamage(0.000035, 0.01, observedDamage, timeuntilbed); //埋没ダメージを計算
        damageCollapse = calcDamage(0.00025, 0.01, observedDamage, timeuntilbed); //崩壊ダメージを計算
        damageFire = calcDamage(0.00025, 0.03, observedDamage, timeuntilbed); //火災ダメージを計算
        totalDamage = damageBury + damageCollapse + damageFire; //合計ダメージを計算

        priority = 1/(timeuntilbed - totalDamage);

        priorityList.put(refugeID, priority);

        //最大値を求める
        if(bestPriority < priority){
            this.bestPriority = priority;
        }
        //最小値を求める
        if(worstPriority > priority){
            this.worstPriority = priority;
        }

        return priorityList;
    }


    //池上
    /**Mapの要素部分をList型に格納する*
     * @param Data 時間別の市民の人数 (要素)
     * @param refugeID 避難所のID (key)
     * @return Map<避難所のID, (int)時間別の市民の人数>
    */
    public Map<EntityID,List<Integer>> packageData(Map<EntityID, Integer> Data, EntityID refugeID){

        Map<EntityID,List<Integer>> packageData = new HashMap<>();
        List<Integer>               list        = new ArrayList<>();

        list.add(Data.get(refugeID));
        packageData.put(refugeID, list);

        return packageData;
    }

    /**
     * マンハッタン距離を移動するのにかかる時間(ステップ数)
     * @param refugeID
     * @param me Agent自身
     * @return int
     */
    private int getManhattanDistance(EntityID refugeID, Human me){
        int normalDistance;//pathのListに含まれているentityの数を格納
        int timecalc = 7000;//マジックナンバー？

        pathPlanning.setFrom(me.getPosition()); //スタート地点
        pathPlanning.setDestination(refugeID);                 //ゴール地点

        //「道順」を取得してリストに加える
        List<EntityID> path = pathPlanning.calc().getResult();
        //パスがnull（Refugeまでのパスがない）なら最大値を返す
        if(path == null){
            return Integer.MAX_VALUE;
        }
        normalDistance      = path.size();

        int pathTime        = normalDistance / timecalc;//マンハッタン距離で移動する時間を格納

        return pathTime;
    }

    //以下　待ち行列の関数
    /**Refugeにいる、市民全員の人数とダメージを受けている市民の人数を取得する
     * @param
     * @param returnName ダメージを受けていない市民を返り値にする場合は____________(string)noDamageCivilians_______________
     * @return
    */
    private Map<EntityID, Integer> getCivilianInfoAtEachRefuge(EntityID refugeID, String returnName){

        Map<EntityID, Integer> eachRefugeNoDamageCiviliansList = new HashMap<>();  // ダメージを受けていない市民の人数
        Map<EntityID, Integer> eachRefugeAllCiviliansList      = new HashMap<>();  // 各避難所の市民の数
        int                    numberOfNoDamageCivilans        = 0;
        int                    numberOfCivilians               = 0;
        Human                  civilian;
        EntityID               civilianPositionID;

        //各避難所にいる市民の情報を取得（Mapに避難所のIDをキーとし、治療が終わった市民の人数を保存する）
        for(EntityID CivilianID : allCivilianIDs){

            sendMessageCivilian(CivilianID, messageManager); //市民の情報を送信

            civilian           = (Human)worldInfo.getEntity(CivilianID);
            civilianPositionID = civilian.getPosition();                 //市民の位置情報を取得

            //市民が避難所にいないなら次のループへ
            if(civilianPositionID != refugeID) {
                continue;
            }

            //避難所にいる市民の人数を数える
            if(eachRefugeAllCiviliansList.containsKey(refugeID)){
                numberOfCivilians = eachRefugeAllCiviliansList.get(refugeID);
            }
            numberOfCivilians = numberOfCivilians + 1;
            eachRefugeAllCiviliansList.put(refugeID, numberOfCivilians);

            //観測した時間ごとにダメージを受けていない市民の人数を調べる
            if(civilian.isDamageDefined()){
                continue;
            }
            if(!eachRefugeNoDamageCiviliansList.containsKey(civilianPositionID)){//初めてRefugeを観測したとき
                numberOfNoDamageCivilans = numberOfNoDamageCivilans + 1;
                eachRefugeNoDamageCiviliansList.put(civilianPositionID, numberOfNoDamageCivilans);
            }
            else{
                numberOfNoDamageCivilans = eachRefugeNoDamageCiviliansList.get(civilianPositionID);
                numberOfNoDamageCivilans = numberOfNoDamageCivilans + 1;
                eachRefugeNoDamageCiviliansList.replace(civilianPositionID, numberOfNoDamageCivilans);
            }
        }
        //ダメージを受けていない市民の人数を返り値に
        if(returnName == "noDamageCivilians"){
            return eachRefugeNoDamageCiviliansList;
        }
        //市民の総数を返り値に
        return eachRefugeAllCiviliansList;
    }

    /**混雑度を計算*
     * @param
    */
    private double getCrowd(EntityID refugeID, String returnName) {
        int           numberOfBeds              = this.numberOfBeds.get(refugeID);
        List<Integer> noDamageCivilianList      = this.noDamageCivilianList.get(refugeID);
        List<Integer> allCiviliansList          = this.allCiviliansList.get(refugeID);
        List<Integer> confirmationTime          = this.confirmationTime.get(refugeID);
        int           listSize                  = noDamageCivilianList.size() - 1;
        double        confirmationTimeLag       = 0; // 時間差
        double        noDamageCivilianVariation = 0; // ConfirmationTimeLagにおけるダメージを受けていない市民の差
        double        allCivilianVariation      = 0; // ConfirmationTimeLagにおける避難所にいる市民の差
        double        crowd;                         // 混雑度
        int           n;                             // for文用

        //データが取れていない避難所の場合
        if(noDamageCivilianList.get(0) == null){
            return 0.2; //ATが市民を運び入れたことがないので、混雑度は0.2とする。
        }
        //複数データが取れていないとき、
        if(noDamageCivilianList.get(1) == null){
            return 0.3; //ATが一度だけ市民を運び入れているので、
        }
        //データが取れているが、差がないとき
        if( noDamageCivilianList.get(listSize) - noDamageCivilianList.get(0) == 0){
            return 0.8; //適当
        }

        //時間ごとの治療が終わった市民の数の差を求める。
        for(n = listSize; n < 0; n--){
            noDamageCivilianVariation = noDamageCivilianList.get(listSize) - noDamageCivilianList.get(listSize - n);
            if(noDamageCivilianVariation > 0){
                break;
            }
        }

        //NoDamageCivilianVariationと同じ時間差で、AllCivilianVariationとConfirmationTimeLagの差をとる


        allCivilianVariation = allCiviliansList.get(listSize) - allCiviliansList.get(listSize - n);
        confirmationTimeLag  = confirmationTime.get(listSize) - confirmationTime.get(listSize - n);


        //単位時間あたりに市民がべットからでる人数
        double TreatmentCivilians_UnitTime = noDamageCivilianVariation/confirmationTimeLag;
        //単位時間あたりに行列に並ぶ人数
        double QueueCivilians_UnitTime     = allCivilianVariation/confirmationTimeLag;
        //混雑度
        crowd = QueueCivilians_UnitTime/(TreatmentCivilians_UnitTime * numberOfBeds);

        if(returnName == "TreatmentCivilians_UnitTime"){
            return TreatmentCivilians_UnitTime;
        }

        if(crowd >= 1 || crowd < 0){ //混雑度がオーバーフローした場合
            return 0.99;
        }

        //classの方の関数に代入
        this.allCivilianVariations = allCivilianVariation;

        return crowd;
    }

    // /**待ち時間の計算(フォーク型待ち行列)**/
    // private double calcWaitingTime(EntityID refugeID){

    //     double waitingTime = 0;
    //     double a           = getCrowd(refugeID, "crowd");
    //     double S           = numberOfBeds.get(refugeID);
    //     double mu          = getCrowd(refugeID, "TreatmentCivilians_UnitTime");

    //     waitingTime = ((Math.pow(a, S)*S/(S - a))/(Sum(a, S-1) + Math.pow(a, S)*S/(S - a)))*(1/(mu*(S - a)));

    //     return waitingTime;
    // }

    /**待ち時間の計算(リトルの公式)**/
    private double calcWaitingTimeLittle(EntityID refugeID){
        double waitingTime;
        List<Integer> lst = this.allCiviliansList.getOrDefault(refugeID, null);
        double allCivilians;
        if(Objects.isNull(lst) || lst.isEmpty()){
            allCivilians = 0;
        }
        else{
            if(lst.get(lst.size()-1) == null){
                allCivilians = 0;
            }else{
                allCivilians = lst.get(lst.size() - 1);
            }
        }
        waitingTime = allCivilians/this.allCivilianVariations;


        return waitingTime;
    }

    /**市民の情報を送信する関数*/
    public void sendMessageCivilian(EntityID civilianID, MessageManager messageManager) {
        Civilian civilian = (Civilian)worldInfo.getEntity(civilianID);
        messageManager.addMessage(new MessageCivilian(true, StandardMessagePriority.NORMAL, civilian));
    }

    /**市民の情報を受信する関数 */
    public void receiveMessageCivilian(MessageManager messageManager) {

        List<CommunicationMessage> messages = messageManager.getReceivedMessageList(MessageCivilian.class);
        for(CommunicationMessage message : messages){
            MessageUtil.reflectMessage(this.worldInfo, (MessageCivilian)message);
        }
    }

     /**Refugeの情報を送信する関数*/
     public void sendMessageRefuge(EntityID refugeID, MessageManager messageManager) {
        Building refuge = (Building)worldInfo.getEntity(refugeID);
        messageManager.addMessage(new MessageBuilding(true, StandardMessagePriority.NORMAL, refuge));
    }

    /**Refugeの情報を受信する関数 */
    public void receiveMessageRefuge(MessageManager messageManager) {

        List<CommunicationMessage> messages = messageManager.getReceivedMessageList(MessageBuilding.class);
        for(CommunicationMessage message : messages){
            MessageUtil.reflectMessage(this.worldInfo, (MessageBuilding)message);
        }
    }

    /**総和の関数**/
    private double Sum(double n, double a){
        double answer = 0;
        for(int i = 0; i <= n; i++){
            answer = answer + Math.pow(a, i)/Factorial(i);
        }
        return answer;
    }

    /**階乗の関数**/
    private double Factorial(int n){
        if(n == 0){
            return 1;
        }
        double answer = 1;
        for(int i = 1; i <= n; i++){
            answer = answer * i;
        }
        return answer;
    }
}