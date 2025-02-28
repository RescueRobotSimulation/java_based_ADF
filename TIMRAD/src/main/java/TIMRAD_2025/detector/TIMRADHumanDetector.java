package TIMRAD_2025.detector;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.complex.HumanDetector;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

import TIMRAD_2025.extaction._RIOHistoryManager;

import static rescuecore2.standard.entities.StandardEntityURN.*;


public class TIMRADHumanDetector extends HumanDetector {

	private Clustering clustering;

	private _RIOHistoryManager history;

	private final static int HUMAN_MAX_HP = 10000;

	private int rescueDistance;
	private final double agentRadius = 500.0; // エージェント自身の半径


	private EntityID result;

	///**** ここから要修正（HistoryManagerに委譲すべき）
	// 座標を一度に返すときに使用
	public class pointXY {
		public int X;
		public int Y;
	}

	// 割り込みの理由
	public enum interruptStatus {
		interruptMoving, NonAction, RepeatedSideJump, NormalProcess;
	}
	///**** ここまで要修正（HistoryManagerに委譲すべき）

	public TIMRADHumanDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);
		this.history = new _RIOHistoryManager();

		this.rescueDistance = si.getClearRepairDistance();
		this.result = null;

		switch (scenarioInfo.getMode()) {
		case PRECOMPUTATION_PHASE:
			this.clustering = moduleManager.getModule("SampleHumanDetector.Clustering", "RIO2023.algorithm.RioneKmeansPP");
			break;
		case PRECOMPUTED:
			this.clustering = moduleManager.getModule("SampleHumanDetector.Clustering", "RIO2023.algorithm.RioneKmeansPP");
			break;
		case NON_PRECOMPUTE:
			this.clustering = moduleManager.getModule("SampleHumanDetector.Clustering", "RIO2023.algorithm.RioneKmeansPP");
			break;
		}
		registerModule(this.clustering);
	}

	@Override
	public HumanDetector updateInfo(MessageManager messageManager) {
		super.updateInfo(messageManager);
		if (this.getCountUpdateInfo() > 1) {
			return this;
		}

		return this;
	}

	@Override
	public HumanDetector calc() {
		// 反復横跳び対策
		Human myself = (Human) this.agentInfo.me();

		// switch (this.shouldInterrupt(myself)) {
			// case interruptMoving:
			// 	history.print("interruptMoving");
			// 	this.result = this.interruptAction();
			// 	this.actionProcess();
			// 	if (this.result != null) {
			// 		return this;
			// 	}
			// 	break;
			// case NonAction:
			// 	history.print("NonAction");
			// 	this.result = this.nonActionProcess();
			// 	if (this.result != null) {
			// 		return this;
			// 	}
			// 	break;
			// case RepeatedSideJump:
			// 	history.print("RepeatedSideJump");
			// 	this.result = this.nonActionProcess();
			// 	if (this.result != null) {
			// 		return this;
			// 	}
			// 	break;
			// case NormalProcess:
			// 	// System.out.printf("NormalProcess\n");
			// 	break;
			// default:
			// }


		Human transportHuman = this.agentInfo.someoneOnBoard();

		if (transportHuman != null) {
			this.result = transportHuman.getID();
			return this;
		}

		if (clustering == null) {
			this.result = this.triage();
			if (this.result != null) {
				return this;
			}
		} else {
			this.result = this.calcTargetInCluster(clustering);
			if (this.result != null) {
				return this;
			}

			this.result = this.triage();
			if (this.result != null) {
				return this;
			}
		}


		// 無意味なresultを返さないかチェック（無意味な場合はnull->extMoveへ）
		this.result = this.resetTarget(this.result);
		/*
		 * もし humanじゃないresultの処理をする場合はここに記述してください by horie
		*/


		//池上  Searchを実行させるため、ランダムで選択はアカン
		// if(this.result == null){
		// 	this.result = this.randomTarget();
		// }
		return this;
	}

	// 割り込み処理を書く
	private EntityID interruptAction() {
		return this.randomTarget();
	}

	// ランダムなターゲットを指定
	private EntityID randomTarget(){
		List<EntityID> Citizens = new ArrayList<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.CIVILIAN));
		Collections.shuffle(Citizens);
		for(EntityID citizen : Citizens){
			if(this.resetTarget(citizen) != null){
				return citizen;
			}
		}

		return null;
	}

	// 瓦礫に対する行動に伴う処理
	// 現在のサイクルでactionClearまたはactionMoveを行ったことを保存するために作成
	private void actionProcess() {
		// if (result != null && result.getClass() == ActionClear.class) {
		// 	history.setClearedFlg();
		// 	// System.out.printf("result = ActionClear java.awt.Desktop.Action result\n");
		// }
	}

	// 割り込みにエラーが発生した場合の処理
	// historyの初期化などをする
	// private Action nonActionProcess(Human human) {
	private EntityID nonActionProcess() {
		return this.randomTarget();
	}

	/**
	 * @return target
	 * @author horie(16th)
	 * 無意味なresultが来た時にnullを返す; calcの煩雑な処理を関数化
	 * nullを返すことで処理がSearchに移行する（詳しくは勉強してください）
	 */

	private EntityID resetTarget(EntityID result){
		if (result == null){
			return result;
		}

		// humanじゃない場合は別の場所で処理してもらう前提
		if (!(this.worldInfo.getEntity(result) instanceof Human)) {
			return result;
		}

		Human target = (Human) this.worldInfo.getEntity(result);
		if(target == null){
			return result;
		}


		if (!target.isHPDefined() || target.getHP() == 0) {
			return null;
		} else if (!target.isPositionDefined()) {
			return null;
		} else if (this.agentInfo.me() instanceof AmbulanceTeam
			&& target.isBuriednessDefined()
			&& target.getBuriedness() > 0
		) {
			return null;
		}

		// refugeにいるときはtargetにしちゃだめ
		StandardEntity targetPosition = this.worldInfo.getEntity(target.getPosition());
		if (targetPosition != null && targetPosition.getStandardURN() == REFUGE) {
			return null;
		}


		StandardEntity position = this.worldInfo.getPosition(target);
		if (position != null) {
			StandardEntityURN positionURN = position.getStandardURN();
			if (positionURN == REFUGE || positionURN == AMBULANCE_TEAM) {
				return null;
			}
		}


		if (this.agentInfo.me() instanceof AmbulanceTeam){
			return result;
		}

		/*** 以下 スコア悪化の疑惑があるためコメントアウト by Horie
		// 以下, FB限定処理(ATはreturn済み)
		// targetの周りにAgentがいるかどうか判定する
		final StandardEntity myself = this.agentInfo.me();

		// 自分がtarget近くにいるなら問題なし
		StandardEntity targetEntity = this.worldInfo.getEntity(this.result);
		if (this.worldInfo.getDistance(targetEntity, myself) <= this.rescueDistance / 2.0 - this.agentRadius) {
			return this.result;
		}

		// 3人以上密集するのは避けたい
		final int THRESHOLD = 3;
		int count = 0;
		Collection<StandardEntity> otherFBs = this.worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE);
		for(StandardEntity otherFB : otherFBs){
			// agentが近くにいればcountをふやす
			if (this.worldInfo.getDistance(otherFB, targetEntity) > this.rescueDistance / 2.0 - this.agentRadius) {
				continue;
			}

			count ++;
			if(THRESHOLD < count){
				return null;
			}
		}
		*/
		return result;


	}

	/**
	 * @return 反復横跳びなどになっているかどうか
	 * @author nakayama(16th), renewed by horie(16th)
	 */

	// 反復横飛びと瓦礫にハマっている状態を検知
	private interruptStatus shouldInterrupt(Human human) {
		history.printHistory();
		// 1サイクル前に瓦礫を除去した場合は履歴を削除
		// actionClearは移動しないため割り込みを行わない
		if (history.isCleared()) {
			history.resetHistory();
			return interruptStatus.NormalProcess;
		}
		pointXY old_position = new pointXY();
		history.savePosition(human.getX(), human.getY());

		// 割り込みで指定された瓦礫へ移動中
		if (history.getInterruptTargetBlockade() != null) {
			return interruptStatus.interruptMoving;
		}

		// 過去2サイクルの座標と現在の座標から最小の移動距離を求める
		// 複数の座標履歴を使うのは反復横飛びを検知するため
		// 活動していないことを検知。actionClearのためにとどまっている場合は除く
		double distanceCheck = (double) Integer.MAX_VALUE;
		old_position.X = history.getHistoryX(0);
		old_position.Y = history.getHistoryY(0);
		if (old_position.X != -1) {
			distanceCheck = this.getDistance(human.getX(), human.getY(), old_position.X, old_position.Y);
			if (distanceCheck < 1) {
				return interruptStatus.NonAction;
			}
		}
		// 反復横飛びを検知
		// 2サイクル前の座標との比較を行う
		old_position.X = history.getHistoryX(1);
		old_position.Y = history.getHistoryY(1);
		if (old_position.X != -1) {
			distanceCheck = this.getDistance(human.getX(), human.getY(), old_position.X, old_position.Y);
			if (distanceCheck < 1) {
				return interruptStatus.RepeatedSideJump;
			}
		}

		// System.out.printf("distans: %f\n", distanceCheck);

		return interruptStatus.NormalProcess;
	}

	// 名前あんまよくない！要修正（by Horie(16th)）
	private double getDistance(double fromX, double fromY, double toX, double toY) {
		double dx = toX - fromX;
		double dy = toY - fromY;
		return Math.hypot(dx, dy);
	}

	/**
	 * @return target
	 * @author takehara(16th)
	 */
	private EntityID triage() {
		/* リストの作成 */
		List<EntityID> level1s = new ArrayList<>();
		List<EntityID> level2s = new ArrayList<>();
		List<EntityID> level3s = new ArrayList<>();
		List<EntityID> level4s = new ArrayList<>();
		List<EntityID> level5s = new ArrayList<>();
		Random random = new Random(this.agentInfo.me().getID().getValue());

		Human human = (Human)this.agentInfo.me();
		ArrayList<EntityID> civilians = new ArrayList<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.CIVILIAN));

		for(EntityID entityC : civilians) {
			/* 救出する市民の優先度決め */
			if(((Human)worldInfo.getEntity(entityC)).isBuriednessDefined() &&
					((Human)worldInfo.getEntity(entityC)).getHP() > 0 &&
					((Human)worldInfo.getEntity(entityC)).getDamage() > 0 &&
					(this.worldInfo.getEntity(((Human)worldInfo.getEntity(entityC)).getPosition())).getStandardURN() != REFUGE){

				// 優先度の計算
				Human humanC = (Human)this.worldInfo.getEntity(entityC);
				int deathTime = humanC.getHP() / humanC.getDamage();

				if(deathTime <= 5) {
					level5s.add(entityC);
				}
				else if(deathTime <= 50) {
					level4s.add(entityC);
				}
				else if(deathTime <= 100) {
					level3s.add(entityC);
				}
				else if(deathTime <= 150){
					level2s.add(entityC);
				}
				else{
					level1s.add(entityC);
				}
			}
		}
		int distance = Integer.MAX_VALUE; //distanceに適当な値を入れる
		EntityID target = null;
		/* 命が危険な市民から助けていく */
		if(!level4s.isEmpty()) {
			for(EntityID entity4 : level4s) {
				if(this.worldInfo.getDistance(this.worldInfo.getEntity(entity4),human) < distance) {

					//「市民」からの距離 distance を更新
					distance = this.worldInfo.getDistance(this.worldInfo.getEntity(entity4),human);

					// この「市民」を目的地とする
					target = entity4;
				}
			}
			// target = level4s.get(random.nextInt(level4s.size()));
		}
		else if(!level3s.isEmpty()) {
			for(EntityID entity3 : level3s) {
				if(this.worldInfo.getDistance(this.worldInfo.getEntity(entity3),human) < distance) {

					//「市民」からの距離 distance を更新
					distance = this.worldInfo.getDistance(this.worldInfo.getEntity(entity3),human);

					// この「市民」を目的地とする
					target = entity3;
				}
			}
			// target = level3s.get(random.nextInt(level3s.size()));
		}
		else if(!level2s.isEmpty()) {
			for(EntityID entity2 : level2s) {
				if(this.worldInfo.getDistance(this.worldInfo.getEntity(entity2),human) < distance) {

					//「市民」からの距離 distance を更新
					distance = this.worldInfo.getDistance(this.worldInfo.getEntity(entity2),human);

					// この「市民」を目的地とする
					target = entity2;
				}
			}
			// target = level2s.get(random.nextInt(level2s.size()));
		}
		else if(!level1s.isEmpty()) {
			for(EntityID entity1 : level1s) {
				if(this.worldInfo.getDistance(this.worldInfo.getEntity(entity1),human) < distance) {

					//「市民」からの距離 distance を更新
					distance = this.worldInfo.getDistance(this.worldInfo.getEntity(entity1),human);

					// この「市民」を目的地とする
					target = entity1;
				}
			}
			// target = level1s.get(random.nextInt(level1s.size()));
		}

		return target;
	}



	private EntityID calcTargetInCluster(Clustering clustering) {
		int clusterIndex = clustering.calc().getClusterIndex(this.agentInfo.getID());
		Collection<StandardEntity> elements = clustering.calc().getClusterEntities(clusterIndex);
		if (elements == null || elements.isEmpty()) {
			return null;
		}

		List<Human> rescueTargets = new ArrayList<>();
		List<Human> priorityRescueTarget = new ArrayList<>(); //優先的に助けるべきHuman
		List<Human> loadTargets = new ArrayList<>();
		ArrayList<Human> level1s = new ArrayList<>();
		ArrayList<Human> level2s = new ArrayList<>();
		ArrayList<Human> level3s = new ArrayList<>();
		ArrayList<Human> level4s = new ArrayList<>();
		ArrayList<Human> level5s = new ArrayList<>();

		for (StandardEntity next : this.worldInfo.getEntitiesOfType(AMBULANCE_TEAM, FIRE_BRIGADE, POLICE_FORCE)) {
			Human h = (Human) next;
			if (this.agentInfo.getID().getValue() == h.getID().getValue()) {
				continue;
			}
			StandardEntity positionEntity = this.worldInfo.getPosition(h);
			if (positionEntity != null && elements.contains(positionEntity) || elements.contains(h)) {
				if (h.isHPDefined() && h.isBuriednessDefined() && h.getHP() > 0 && h.getBuriedness() > 0) {
					if (h.getHP() < TIMRADHumanDetector.HUMAN_MAX_HP / 2) {
						priorityRescueTarget.add(h);
					} else {
						rescueTargets.add(h);
					}
				}
			}
		}
		for (StandardEntity next : this.worldInfo.getEntitiesOfType(CIVILIAN)) {
			Human h = (Human) next;
			StandardEntity positionEntity = this.worldInfo.getPosition(h);
			if (positionEntity instanceof Area) {
				if (elements.contains(positionEntity)) {
					if (h.isHPDefined() && h.getHP() > 0) {
						if (h.isBuriednessDefined() && h.getDamage() > 0) {

							// 優先度の計算
							Human humanC = (Human)(next);
							int deathTime = humanC.getHP() / humanC.getDamage();

							if(deathTime <= 5) {
								level5s.add(h);
							}
							else if(deathTime <= 50) {
								level4s.add(h);
							}
							else if(deathTime <= 100) {
								level3s.add(h);
							}
							else if(deathTime <= 150) {
								level2s.add(h);
							}
							else{
								level1s.add(h);
							}
						}
						else {
							if (h.isDamageDefined() && h.getDamage() > 0 && positionEntity.getStandardURN() != REFUGE) {
								loadTargets.add(h);
							}
						}
					}
				}
			}
		}
		if (level4s.size() > 0) {
			level4s.sort(new DistanceSorter(this.worldInfo, this.agentInfo.me()));
			return level4s.get(0).getID();
		}
		if (level3s.size() > 0) {
			level3s.sort(new DistanceSorter(this.worldInfo, this.agentInfo.me()));
			return level3s.get(0).getID();
		}
		if(level2s.size() > 0) {
			level2s.sort(new DistanceSorter(this.worldInfo, this.agentInfo.me()));
			return level2s.get(0).getID();
		}
		if(level1s.size() > 0) {
			level1s.sort(new DistanceSorter(this.worldInfo, this.agentInfo.me()));
			return level1s.get(0).getID();
		}
		if (loadTargets.size() > 0) {
			loadTargets.sort(new DistanceSorter(this.worldInfo, this.agentInfo.me()));
			return loadTargets.get(0).getID();
		}
		return null;
	}

	/**
	 * @return targetloadTargets
	 * @author sakaue(15th)
	 * @author amada(14th)
	 */
	private EntityID calcTargetInWorld() {
		List<Human> rescueTargets = new ArrayList<>();
		List<Human> priorityRescueTarget = new ArrayList<>(); //優先的に助けるべきHuman
		List<Human> loadTargets = new ArrayList<>();
		for (StandardEntity next : this.worldInfo.getEntitiesOfType(AMBULANCE_TEAM, FIRE_BRIGADE, POLICE_FORCE)) {
			Human h = (Human) next;
			if (this.agentInfo.getID().getValue() != h.getID().getValue()) {
				StandardEntity positionEntity = this.worldInfo.getPosition(h);
				if (positionEntity != null && h.isHPDefined() && h.isBuriednessDefined()) {
					if (h.getHP() > 0 && h.getBuriedness() > 0) {
						if (h.getHP() < TIMRADHumanDetector.HUMAN_MAX_HP / 2) {
							priorityRescueTarget.add(h);
						} else {
							rescueTargets.add(h);
						}
					}
				}
			}
		}
		for (StandardEntity next : this.worldInfo.getEntitiesOfType(CIVILIAN)) {
			Human h = (Human) next;
			StandardEntity positionEntity = this.worldInfo.getPosition(h);
			if (positionEntity instanceof Area) {
				if (h.isHPDefined() && h.getHP() > 0) {
					if (h.isBuriednessDefined() && h.getBuriedness() > 0) {
						if (h.getHP() < TIMRADHumanDetector.HUMAN_MAX_HP / 2) {
							priorityRescueTarget.add(h);
						} else {
							rescueTargets.add(h);
						}
					} else {
						if (h.isDamageDefined() && h.getDamage() > 0 && positionEntity.getStandardURN() != REFUGE) {
							loadTargets.add(h);
						}
					}
				}
			}
		}
		if (priorityRescueTarget.size() > 0) {
			priorityRescueTarget.sort(new DistanceSorter(this.worldInfo, this.agentInfo.me()));
			return priorityRescueTarget.get(0).getID();
		}
		if (rescueTargets.size() > 0) {
			rescueTargets.sort(new DistanceSorter(this.worldInfo, this.agentInfo.me()));
			return rescueTargets.get(0).getID();
		}
		if (loadTargets.size() > 0) {
			loadTargets.sort(new DistanceSorter(this.worldInfo, this.agentInfo.me()));
			return loadTargets.get(0).getID();
		}
		return null;
	}

	@Override
	public EntityID getTarget() {
		return this.result;
	}

	@Override
	public HumanDetector precompute(PrecomputeData precomputeData) {
		super.precompute(precomputeData);
		if (this.getCountPrecompute() >= 2) {
			return this;
		}
		return this;
	}

	@Override
	public HumanDetector resume(PrecomputeData precomputeData) {
		super.resume(precomputeData);
		if (this.getCountResume() >= 2) {
			return this;
		}
		return this;
	}

	@Override
	public HumanDetector preparate() {
		super.preparate();
		if (this.getCountPreparate() >= 2) {
			return this;
		}
		return this;
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

	private class IdDistanceSorter implements Comparator<EntityID> {
		private EntityID reference;
		private WorldInfo worldInfo;

		IdDistanceSorter(WorldInfo wi, EntityID reference) {
			this.reference = reference;
			this.worldInfo = wi;
		}

		public int compare(EntityID a, EntityID b) {
			int d1 = this.worldInfo.getDistance(this.reference, a);
			int d2 = this.worldInfo.getDistance(this.reference, b);
			return d1 - d2;
		}
	}
}

