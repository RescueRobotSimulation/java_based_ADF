package TIMRAD_2025._taskmanager;

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
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.complex.RoadDetector;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

/**
 * @author Horie(16th)
 * @since 2020.10.- summary : 動的なtarget選択のために, Comparatorと優先度管理クラスを実装 優先度と,
 *        優先度の変化しやすさ（＝不安定度）を各対象ごとに求め管理する
 *
 */

public class RIODynamicTaskManager {

    //// attention : オーバーフロー対策必要？
    //private int insertOrder = 0; // 挿入順

    /// 情報
    private ScenarioInfo scenarioInfo;
    private WorldInfo    worldInfo;
    private AgentInfo    agentInfo;
    private PathPlanning pathPlanning;
    private Clustering   clustering;
    private MessageManager messageManager;

    /// target候補管理
    private Set<IRIOEvaluator> evaluators                      = new HashSet<IRIOEvaluator>();         //!< 評価基準リスト
    private Set<RIODynamicTaskKey> prioritySet                 = new HashSet<RIODynamicTaskKey>();
    private PriorityQueue<RIODynamicTaskKey> unstablenessQueue = new PriorityQueue<RIODynamicTaskKey>(
        new UnstablenessChecker()
    );
    private Map<EntityID, Integer> evaluateeTimes = new HashMap<>(); //!< 登録してからの経過時間

    /// ---- 内部クラス ----
    /// summary : 不安定度管理のための優先度付きキュー管理クラス
    private class UnstablenessChecker implements Comparator<RIODynamicTaskKey> {
        public int compare(RIODynamicTaskKey key1, RIODynamicTaskKey key2) {
            // 不安定度を比較
            if (key1.unstableness < key2.unstableness){
                return 1;
            }
            if (key1.unstableness > key2.unstableness){
                return -1;
            }
            return 0;
        }
    }

    /// summary : target決定のための優先度付きキュー管理クラス
    private class PriorityChecker implements Comparator<RIODynamicTaskKey> {
        public int compare(RIODynamicTaskKey key1, RIODynamicTaskKey key2) {
            // 評価値を比較
            if (key1.priority < key2.priority){
                return 1;
            }
            if (key1.priority > key2.priority){
                return -1;
            }
            return 0;
        }
    }

    /// ---- メンバメソッド ----
    /// この順番に実行しないと実行時エラーを招きます（要検討）
    /// summary : ビルダーメソッド（Info系）
    public RIODynamicTaskManager setInfo(ScenarioInfo scenarioInfo,
                                         WorldInfo worldInfo,
                                         AgentInfo agentInfo       ){
        // 情報の取得
        this.scenarioInfo = scenarioInfo;
        this.worldInfo = worldInfo;
        this.agentInfo = agentInfo;
        return this;
    }

    /// summary ビルダーメソッド（アルゴリズム系）
    public RIODynamicTaskManager setAlgorithm(PathPlanning pathPlanning, Clustering clustering){
        this.pathPlanning = pathPlanning;
        this.clustering   = clustering;
        return this;
    }

    ///summary ビルダーメソッド
    public RIODynamicTaskManager setMessageManager(MessageManager messageManager){
        this.messageManager = messageManager;
        return this;
    }

    /// summary : ビルダーメソッド（評価対象）
    public RIODynamicTaskManager setTargetKind(StandardEntityURN ... types){
        // prioritySetのセットアップ
        for(StandardEntityURN type : types){
            Collection<EntityID> allTargets = this.worldInfo.getEntityIDsOfType(type);
            for(EntityID id : allTargets){
                this.insertPropergationError(id);
                this.evaluateeTimes.put(id, 0);
            }
        }
        return this;
    }

    /// summary : 評価基準の追加
    public RIODynamicTaskManager setEvaluator(IRIOEvaluator evaluator) {
        evaluator.setInfo(this.scenarioInfo, this.worldInfo, this.agentInfo)
                 .setAlgorithm(this.pathPlanning, this.clustering);
        this.evaluators.add(evaluator);

        // insertPropagationError 用の処理
        this.observeNs.put(evaluator, 0);

        return this;
    }


    /// 評価基準の統合
    private final static double OBSERVE_THERSHOLD = 0.3; //!< 事象が観測されたとみなす最低の優先度
    private Map<IRIOEvaluator, Integer> observeNs = new HashMap<>();

    /**
     * @brief  微分方程式と誤差伝播則による優先度・不安定度の統合
     * @param  keyID ターゲットのEnitityID
     * @return RIODynamicTaskmanager this を想定
     * @author Horie(16th)
     * @since  2021.11.
     * @note   @par 導出式は以下のように根拠づけられている.
     *   - \f$\mu _k       \cdots\f$ 基準\f$k\f$による候補の優先度の平均
     *   - \f$\sigma_k     \cdots\f$ 基準\f$k\f$による不安定度
     *   - \f$p_k          \cdots\f$ 観測されるうる優先度
     *   - \f$n_k          \cdots\f$ 基準\f$k\f$にあてはまる観測数
     *   - \f$t            \cdots\f$ 計算対象の候補の優先度/不安定度が更新されていないサイクル数
     *   - \f$P            \cdots\f$ 観測されるうる統合後優先度
     *   - \f$\overline{P} \cdots\f$ 統合後の優先度の平均
     *   - \f$U            \cdots\f$ 統合後の不安定度
     *
     *   まず, 優先度が以下のように計算されることを仮定する.
     *   \f{align}
     *      P & = \sum_{k}^{} p_k \log (n_k + 2)
     *   \f}
     *   すなわち, 観測数が多い事象ほど優先度が高くなるという仮説に基づいた計算である.
     *   また, 各基準の優先度, 統合後の基準がそれぞれ以下の正規分布に従うことも仮定する.
     *   \f{align}
	 *      p_k & \sim \mathcal{N} (\mu_k , (\sigma_k \log (t + 2))^2) \\
	 *      P & \sim \mathcal{N} (\overline{P}, U^2)
     *   \f}
     *
     *   この際, 統合後の優先度の平均は, 当然
     *   \f{align}
     *      \overline{P} & = \sum_{k}^{} \mu_k \log (n_k + 2)
     *   \f}
     *   となる. 一方, 統合後の不安定度は, 誤差伝播則より, \f$\mu_k\f$の誤差を\f$\delta\mu_k\f$とすれば
     *   \f{align}
	 *   U^2  & = \sum_{k}^{} \left(\frac{d}{d\mu_t}\overline{P} \right)^2 \cdot \delta\mu^2_k                                                          \\
	 *        & = \sum_{k}^{} (\log(n_k + 2) \cdot \sigma_k \log (t + 2))^2 \hspace{1cm}(∵ \delta\mu_k = \sigma_k \log (t + 2)) \\
	 *   U    & = \sqrt{\sum_{k}^{} \left((\log(n_k + 2))^2 \cdot \sigma_k^2 (\log (t + 2))^2 \right)}
     *   \f}
     *   として求められる.
     */
    private RIODynamicTaskManager insertPropergationError(EntityID keyID) {
        // Map<IRIOEvaluator, Double> priorities     = new HashMap<>();
        // Map<IRIOEvaluator, Double> unstablenesses = new HashMap<>();

        // 評価値と不安定度の計算
        double priority         = 0;
        double unstablenessSqr = 0;

        // 基準ごとの記録計算
        for (IRIOEvaluator taskEvaluator : evaluators) {
            RIODynamicTaskKey result = taskEvaluator.calcNormalized(keyID);

            // 観測回数の記録
            int observedN = this.observeNs.get(taskEvaluator);
            if(priority > OBSERVE_THERSHOLD){
                this.observeNs.replace(taskEvaluator, observedN);
            }

            // 優先度・不安定度計算
            int time = this.evaluateeTimes.get(keyID);
            priority        += result.priority * Math.log(observedN + 2);
            unstablenessSqr += Math.pow(result.unstableness * Math.log(time + 2) * Math.log(observedN + 2), 2);
        }

        // 各コレクションへのタスクの登録
        RIODynamicTaskKey nowKey = new RIODynamicTaskKey(keyID, priority, Math.sqrt(unstablenessSqr));
        this.prioritySet.add(nowKey);
        this.unstablenessQueue.add(nowKey);
        return this;
    }


    /**
     * @deprecated
     * @brief  単純な線形結合による優先度・不安定度の統合
     * @param  keyID ターゲットのEnitityID
     * @return RIODynamicTaskmanager this を想定
     * @author Horie(16th)
     * @since  2021.11.
     * @attention 非推奨. あくまでも初期仕様であり, 一切数学的根拠はない.
     * @Deprecated
     */
    private RIODynamicTaskManager insertSimpleLinear(EntityID keyID) {
        // 時間経過による不安定度の低減（編集中）

        // 評価値と不安定度の計算
        double priority = 0;
        double unstableness = 0;

        // 要修正
        for (IRIOEvaluator taskEvaluator : evaluators) {
            RIODynamicTaskKey result = taskEvaluator.calcNormalized(keyID);
            priority     += result.priority;
            unstableness += result.unstableness;
        }

        // 各コレクションへのタスクの登録
        RIODynamicTaskKey nowKey = new RIODynamicTaskKey(keyID, priority, unstableness);
        this.prioritySet.add(nowKey);
        this.unstablenessQueue.add(nowKey);
        return this;
    }

    /// summary : 不安定度が高いキーの優先度再評価
    public RIODynamicTaskManager update() {
        // 各Evaluatorのupdate
        for (IRIOEvaluator taskEvaluator : evaluators) {
            taskEvaluator.update();
        }

        // 各候補の時間update
        for (EntityID id : this.evaluateeTimes.keySet()){
            int currentTime = this.evaluateeTimes.get(id);
            this.evaluateeTimes.replace(id, currentTime);
        }

        // コレクションからの取り出し・削除
        RIODynamicTaskKey updateeKey = this.unstablenessQueue.poll();
        prioritySet.remove(updateeKey);

        // 評価値の再付与
        this.insertPropergationError(updateeKey.key);
        this.evaluateeTimes.replace (updateeKey.key, 0);
        return this;
    }

    /// summary : target決定処理の実行
    //// 取り出し後の処理検討中
    public EntityID executation() {
        RIODynamicTaskKey ans = Collections.max(prioritySet, new PriorityChecker());
        return ans.key;
    }
}
