package TIMRAD_2025.extaction;

import adf.core.agent.action.Action;
import adf.core.agent.action.common.ActionMove;
import adf.core.agent.action.common.ActionRest;
import adf.core.agent.action.police.ActionClear;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.extaction.ExtAction;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import jdk.jfr.Unsigned;

import com.google.common.collect.Lists;
import rescuecore2.config.NoSuchConfigOptionException;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Vector2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.awt.*;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;

import javax.naming.spi.DirStateFactory.Result;

import static rescuecore2.standard.entities.StandardEntityURN.REFUGE;


// 位置情報を保存しておくクラス
public class TIMRADHistoryManager {
	private int[] pointHistoryArrayX;// X位置情報 [0]は1サイクル前の情報 [1]は2サイクル前
	private int[] pointHistoryArrayY;// Y位置情報[
	private int currentPositionX;// 現在の座標
	private int currentPositionY;
	private final int historySize = 2;// 保存期間
	private int historyUsage;// 配列の使用数
	private boolean isCleared;// 1サイクル前にactionClearをしたか？
	private Blockade targetBlockade;// 割り込みによって設定した瓦礫 設定されている間は探索を行わず、最短でreturnする
	private final boolean debug = false; // エージェントの座標を表示

	public TIMRADHistoryManager() {
		// this.historySize = 2;// 反復横飛びを検知するには過去2サイクルの位置情報があれば検出可能
		this.pointHistoryArrayX = new int[this.historySize];
		this.pointHistoryArrayY = new int[this.historySize];
		this.currentPositionX = this.currentPositionY = -1;
		this.isCleared = false;
		this.targetBlockade = null;
		// -1で初期化(座標は負にならないため)
		for (int i = 0; i < this.historySize; i++) {
			this.pointHistoryArrayX[i] = this.pointHistoryArrayY[i] = -1;
		}
	}

	// 位置情報を保存
	public boolean savePosition(int currentX, int currentY) {
		if (currentX < 0 || currentY < 0) {
			return false;
		}
		int nonUsedIdx = 1;// historySizeはidx-1なので配列番号簡単のため1で初期化
		// 位置情報を更新
		for (int i = 0; i < this.historySize - 1; i++) {
			this.pointHistoryArrayX[i + 1] = this.pointHistoryArrayX[i];
			this.pointHistoryArrayY[i + 1] = this.pointHistoryArrayY[i];
		}
		// 1サイクル前に最新だった座標を配列へ格納
		this.pointHistoryArrayX[0] = this.currentPositionX;
		this.pointHistoryArrayY[0] = this.currentPositionY;
		// 現在の座標に更新
		this.currentPositionX = currentX;
		this.currentPositionY = currentY;
		// まだ未使用の配列を使用した場合
		if (this.historyUsage < this.historySize - 1) {
			this.historyUsage++;
		}

		return true;
	}

	public void resetHistory() {
		for (int i = 0; i < this.historySize; i++) {
			this.pointHistoryArrayX[i] = -1;
			this.pointHistoryArrayY[i] = -1;
		}
		this.currentPositionX = this.currentPositionY = -1;
		this.historyUsage = -1;
		this.isCleared = false;
		this.targetBlockade = null;
		return;
	}

	public int getHistoryX(int idx) {
		if (idx > this.historySize || idx < 0) {
			return -1;
		}
		if (this.pointHistoryArrayX[idx] == -1) {
			return -1;
		}
		return this.pointHistoryArrayX[idx];
	}

	public int getHistoryY(int idx) {
		if (idx > this.historySize || idx < 0) {
			return -1;
		}
		if (this.pointHistoryArrayY[idx] == -1) {
			return -1;
		}
		return this.pointHistoryArrayY[idx];
	}

	public int getHistorySize() {
		return this.historySize;
	}

	public int getHistoryUsage() {
		return this.historyUsage;
	}

	public void setClearedFlg() {
		this.isCleared = true;
	}

	public boolean isCleared() {
		return this.isCleared;
	}

	public void setInterruptTargetBlockade(Blockade targetBlockade) {
		this.targetBlockade = targetBlockade;
	}

	public void resetInterruptTargetBlockade() {
		this.targetBlockade = null;
	}

	public Blockade getInterruptTargetBlockade() {
		return this.targetBlockade;
	}

	// 配列の表示 デバッグ用
	public void printHistory() {
		if (this.debug) {
			System.out.printf("\n");
			System.out.printf("currentPosition: %d, %d\n", this.currentPositionX, this.currentPositionY);
			for (int i = 0; i < this.getHistorySize(); i++) {
				System.out.printf("history[%d]: %d, %d\n", i, this.getHistoryX(i), this.getHistoryY(i));
			}
			System.out.printf("\n");
		}
	}

	public void print(String str) {
		if (this.debug) {
			System.out.printf("%s\n", str);
		}
	}
}
