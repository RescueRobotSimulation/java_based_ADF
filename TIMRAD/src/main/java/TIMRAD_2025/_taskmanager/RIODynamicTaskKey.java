package TIMRAD_2025._taskmanager;

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

import TIMRAD_2025._taskmanager.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;

/**
 * @author Horie(16th)
 * @since 2020.10.-
 * /// summary : 各基準ごとに, 優先度と優先度の変わりやすさ（不安定度）を算出するクラス
 *
 * 以下, 注意事項(by Horie)
 *
 * 1.  抽象クラスであるので, これを継承して評価値計算クラスを作る
 * 2.  実際に使うときは, 具象クラスではなくRIODynamicTaskEvaluatorのインスタンスとして各インスタンスを定義する
 * 3.  bestPriority() < worstPriority()である必要はない.
 *     考えた評価基準の意味に沿って実装してくれればnormalizeがうまくやってくれる.
 * 4.  finalやabstractで制限をかけているものにはそれだけの理由があるので制限は外さないで！
 */

/// summary : 優先度付きキューのキーと優先度をセットにしたクラス
public class RIODynamicTaskKey {
    public final EntityID key;
    public final double priority; // 評価値
    public final double unstableness; // 不安定度

    public RIODynamicTaskKey(EntityID argKey, double priority, double unstableness) {
        this.key = argKey;
        this.priority = priority;
        this.unstableness = unstableness;
    }
}