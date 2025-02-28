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

public class TIMRADActionExtClear extends ExtAction {
	private PathPlanning pathPlanning;

	private int clearDistance;
	private int forcedMove;
	private int thresholdRest;
	private int kernelTime;

	private EntityID target;
	private Map<EntityID, Set<Point2D>> movePointCache;
	private int oldClearX;
	private int oldClearY;
	private int count;

	// for calcClear
	private ArrayList<Pair<Integer, Integer>> removedQueue;
	private final int queueLength = 10;
	private int checkedTime = -1;

	// for calcSearch
	private Clustering clustering;
	private ArrayList<Integer> movedTime;
	private boolean stopped;
	private ArrayList<EntityID> unsearchedBuildingIDs;
	private int clusterIndex;
	private int changeClusterCycle;
	protected Random random;
	private ArrayList<Point2D> previousLocations;
	private ArrayList<List<EntityID>> previousPaths;
	private ArrayList<EntityID> previousTarget;

	// for fDE
	private final double agentRadius = 500.0; 
	// [Debug Code]
	private Blockade formerTarget = null;

	public class pointXY {
		public int X;
		public int Y;
	}

	public enum interruptStatus {
		interruptMoving, NonAction, RepeatedSideJump, NormalProcess;
	}


	private TIMRADHistoryManager history;

	public static final int NON_POSITION = -1;

	public TIMRADActionExtClear(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager,
			DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);
		this.clearDistance = si.getClearRepairDistance();
		this.forcedMove = developData.getInteger("ActionExtClear.forcedMove", 3);
		this.thresholdRest = developData.getInteger("ActionExtClear.rest", 100);

		this.target = null;
		this.movePointCache = new HashMap<>();
		this.oldClearX = 0;
		this.oldClearY = 0;
		this.count = 0;

		switch (si.getMode()) {
		case PRECOMPUTATION_PHASE:
			this.pathPlanning = moduleManager.getModule("DefaultExtActionClear.PathPlanning",
					"adf.core.component.module.algorithm.PathPlanning");
			this.clustering = moduleManager.getModule("DefaultExtActionClear.Clustering.Police",
					"TIMRAD_2025.algorithm.TIMRADKmeansPP");
			break;
		case PRECOMPUTED:
			this.pathPlanning = moduleManager.getModule("DefaultExtActionClear.PathPlanning",
					"adf.core.component.module.algorithm.PathPlanning");
			this.clustering = moduleManager.getModule("DefaultExtActionClear.Clustering.Police",
					"TIMRAD_2025.algorithm.TIMRADKmeansPP");
			break;
		case NON_PRECOMPUTE:
			this.pathPlanning = moduleManager.getModule("DefaultExtActionClear.PathPlanning",
					"adf.core.component.module.algorithm.PathPlanning");
			this.clustering = moduleManager.getModule("DefaultExtActionClear.Clustering.Police",
					"TIMRAD_2025.algorithm.TIMRADKmeansPP");
			break;
		}

		// calcSearch用
		this.clustering = moduleManager.getModule("ActionTransport.Clustering.Ambulance",
				"RIO2023.algorithm.RioneKmeansPP");
		unsearchedBuildingIDs = new ArrayList<>();
		movedTime = new ArrayList<>();
		this.changeClusterCycle = 5;
		this.clusterIndex = 0;
		this.random = new Random();
		this.stopped = false;
		this.previousLocations = new ArrayList<>();
		this.previousPaths = new ArrayList<>();
		this.previousTarget = new ArrayList<>();

		// 自作クラス
		// 位置情報を履歴を保存
		this.history = new TIMRADHistoryManager();

		removedQueue = new ArrayList<>();
	}

	@Override
	public ExtAction precompute(PrecomputeData precomputeData) {
		super.precompute(precomputeData);
		if (this.getCountPrecompute() >= 2) {
			return this;
		}
		this.pathPlanning.precompute(precomputeData);
		try {
			this.kernelTime = this.scenarioInfo.getKernelTimesteps();
		} catch (NoSuchConfigOptionException e) {
			this.kernelTime = -1;
		}
		return this;
	}
//
	@Override
	public ExtAction resume(PrecomputeData precomputeData) {
		super.resume(precomputeData);
		if (this.getCountResume() >= 2) {
			return this;
		}
		this.pathPlanning.resume(precomputeData);
		try {
			this.kernelTime = this.scenarioInfo.getKernelTimesteps();
		} catch (NoSuchConfigOptionException e) {
			this.kernelTime = -1;
		}
		return this;
	}

	@Override
	public ExtAction preparate() {
		super.preparate();
		if (this.getCountPreparate() >= 2) {
			return this;
		}
		this.pathPlanning.preparate();
		try {
			this.kernelTime = this.scenarioInfo.getKernelTimesteps();
		} catch (NoSuchConfigOptionException e) {
			this.kernelTime = -1;
		}
		return this;
	}

	private void reset() {
		this.unsearchedBuildingIDs.clear();
		this.previousPaths.clear();
		this.previousLocations.clear();

		if ((this.agentInfo.getTime() != 0 && (this.agentInfo.getTime() % this.changeClusterCycle) == 0) || stopped) {
			this.stopped = false;
			this.clusterIndex = random.nextInt(clustering.getClusterNumber());
			this.changeClusterCycle = random.nextInt(16) + 15;// 変更

		}
		Collection<StandardEntity> clusterEntities = new ArrayList<>();
		if (clustering != null) {
			clusterEntities.addAll(this.clustering.getClusterEntities(clusterIndex));
		}

		if (clusterEntities.size() > 0) {
			for (StandardEntity entity : clusterEntities) {
				if (entity instanceof Building && entity.getStandardURN() != REFUGE) {
					this.unsearchedBuildingIDs.add(entity.getID());
				}
			}
		} else {
			this.unsearchedBuildingIDs.addAll(this.worldInfo.getEntityIDsOfType(StandardEntityURN.BUILDING));
		}
	}

	@Override
	public ExtAction updateInfo(MessageManager messageManager) {
		super.updateInfo(messageManager);
		if (this.getCountUpdateInfo() >= 2) {
			return this;
		}
		this.pathPlanning.updateInfo(messageManager);

		if (this.unsearchedBuildingIDs.isEmpty()) {
			this.reset();
		}

		// 
		List<EntityID> perceivedBuildings = new ArrayList<>();// 見つけた建物
		for (EntityID id : worldInfo.getChanged().getChangedEntities()) {
			StandardEntity se = worldInfo.getEntity(id);
			if (se instanceof Building) {
				perceivedBuildings.add(id);
			}
		}
		for (EntityID pID : perceivedBuildings) {
			unsearchedBuildingIDs.remove(pID);
		}
		return this;
	}

	@Override
	public ExtAction setTarget(EntityID target) {
		this.target = null;
		StandardEntity entity = this.worldInfo.getEntity(target);
		if (entity != null) {
			if (entity instanceof Road) {
				this.target = target;
			} else if (entity.getStandardURN().equals(StandardEntityURN.BLOCKADE)) {
				this.target = ((Blockade) entity).getPosition();
			} else if (entity instanceof Building) {
				this.target = target;
			}
			this.target = makeTwoManCell(target);
			previousTarget.add(this.target);
		}
		return this;
	}

	/**
	 * @author Amada(14th)
	 * @since 2019 いまのところAreaクラスしか受け取れない (受け取っても無視する)
	 */
	@Override
	public ExtAction calc() {
		this.result = null;
		PoliceForce policeForce = (PoliceForce) this.agentInfo.me();

		//Ikegami追記
		List<StandardEntity> blockades = new ArrayList<>(this.worldInfo.getEntitiesOfType(StandardEntityURN.BLOCKADE));
		if (isStopping() && !blockades.isEmpty()) {
			Random r = new Random();
			this.result = new ActionClear((Blockade)blockades.get(r.nextInt(blockades.size())));
			return this;
		}

		switch (this.shouldInterrupt(policeForce)) {
		case interruptMoving:
			history.print("interruptMoving");
			this.result = this.interruptAction(policeForce);
			this.actionProcess(this.result);
			if (this.result != null) {
				return this;
			}
			break;
		case NonAction:
			history.print("NonAction");
			this.result = this.nonActionProcess(policeForce);
			if (this.result != null) {
				return this;
			}
			break;
		case RepeatedSideJump:
			history.print("RepeatedSideJump");
			this.result = this.nonActionProcess(policeForce);
			if (this.result != null) {
				return this;
			}
			break;
		case NormalProcess:
			// System.out.printf("NormalProcess\n");
			break;
		default:
		}

		// 
		StandardEntity me = this.agentInfo.me();
		if (isInExtendedBlockade((Human) me)) {
			StandardEntity pos = this.worldInfo.getPosition(me.getID());
			if (pos instanceof Road) {
				Blockade blockade = blockadeWithHuman((Human) me);
				if (blockade != null) {
					this.result = new ActionClear(blockade);
					this.actionProcess(this.result);
					return this;
				}
			}
		}
		if (this.needRest(policeForce)) {
			List<EntityID> list = new ArrayList<>();
			if (this.target != null) {
				list.add(this.target);
			}
			this.result = this.calcRest(policeForce, this.pathPlanning, list);
			if (this.result != null) {
				this.actionProcess(this.result);
				return this;
			}
		}

		if (this.target == null) {
			return this;
		}
		EntityID agentPosition = policeForce.getPosition();
		StandardEntity targetEntity = this.worldInfo.getEntity(this.target);
		StandardEntity positionEntity = Objects.requireNonNull(this.worldInfo.getEntity(agentPosition));
		if (!(targetEntity instanceof Area)) {
			this.actionProcess(this.result);
			return this;
		}
		if (positionEntity instanceof Road) {
			this.result = this.removeDeadEnd(); // 優先啓開
			this.actionProcess(this.result);
			if (this.result != null) {
				return this;
			}

			if (!((Road) positionEntity).isBlockadesDefined()) {
				this.result = this.getRescueAction(policeForce, (Road) positionEntity);
			}
			if (this.result != null) {
				return this;
			}
		}
		if (agentPosition.equals(this.target)) {
			this.result = this.getAreaClearAction(policeForce, targetEntity);
		} else if (((Area) targetEntity).getEdgeTo(agentPosition) != null) {
			this.result = this.getNeighbourPositionAction(policeForce, (Area) targetEntity);
		} else {
			List<EntityID> path = this.pathPlanning.getResult(agentPosition, this.target);
			if (path != null && path.size() > 0) {
				int index = path.indexOf(agentPosition);
				if (index == -1) {
					Area area = (Area) positionEntity;
					for (int i = 0; i < path.size(); i++) {
						if (area.getEdgeTo(path.get(i)) != null) {
							index = i;
							break;
						}
					}
				} else {
					index++;
				}
				if (index >= 0 && index < (path.size())) {
					StandardEntity entity = this.worldInfo.getEntity(path.get(index));
					this.result = this.getNeighbourPositionAction(policeForce, (Area) entity);
					this.actionProcess(this.result);
					if (this.result != null && this.result.getClass() == ActionMove.class) {
						if (!((ActionMove) this.result).getUsePosition()) {
							this.result = null;
						}
					}
				}
				if (this.result == null) {
					this.result = new ActionMove(path);
					this.actionProcess(this.result);
				}
			}
		}
		this.actionProcess(this.result);
		// /* ここから */
		// this.result = this.processTargetDecided(policeForce);
		// this.actionClearProcess(this.result);
		// return this;
		// this.actionClearProcess(this.result);
		return this;
	}

	private interruptStatus shouldInterrupt(PoliceForce policeForce) {
		history.printHistory();
		// actionClear
		if (history.isCleared()) {
			history.resetHistory();
			return interruptStatus.NormalProcess;
		}
		pointXY old_position = new pointXY();
		history.savePosition(policeForce.getX(), policeForce.getY());

		// 
		if (history.getInterruptTargetBlockade() != null) {
			return interruptStatus.interruptMoving;
		}

		// 
		double distanceCheck = (double) Integer.MAX_VALUE;
		old_position.X = history.getHistoryX(0);
		old_position.Y = history.getHistoryY(0);
		if (old_position.X != -1) {
			distanceCheck = this.getDistance(policeForce.getX(), policeForce.getY(), old_position.X, old_position.Y);
			if (distanceCheck < 1) {
				return interruptStatus.NonAction;
			}
		}
		// 
		// 
		old_position.X = history.getHistoryX(1);
		old_position.Y = history.getHistoryY(1);
		if (old_position.X != -1) {
			distanceCheck = this.getDistance(policeForce.getX(), policeForce.getY(), old_position.X, old_position.Y);
			if (distanceCheck < 1) {
				return interruptStatus.RepeatedSideJump;
			}
		}

		// System.out.printf("distans: %f\n", distanceCheck);

		return interruptStatus.NormalProcess;
	}

	// 
	private Action interruptAction(PoliceForce policeForce) {
		if (history.getInterruptTargetBlockade() == null) {
			history.resetHistory();
			return null;
		}
		Blockade target = history.getInterruptTargetBlockade();
		double distance = this.getBlockadeDistance(policeForce, target);
		if (distance < this.clearDistance / 2) {
			return new ActionClear(target);
		} else {
			return this.getMovePath(target);
		}
	}

	private Action nonActionProcess(PoliceForce policeForce) {
		Action result = clear2NearestBlockade(policeForce);
		this.actionProcess(result);
		if (result == null) {
			history.resetHistory();
			return null;
		}
		return result;
	}

	// 瓦礫に対する行動に伴う処理
	// 現在のサイクルでactionClearまたはactionMoveを行ったことを保存するために作成
	private void actionProcess(Action result) {
		if (result != null && result.getClass() == ActionClear.class) {
			history.setClearedFlg();
			// System.out.printf("result = ActionClear java.awt.Desktop.Action result\n");
		}
	}

	

	// 
	private Action clear2NearestBlockade(PoliceForce policeForce) {
		double nearest = Double.MAX_VALUE;
		StandardEntity positionEntity = this.worldInfo.getPosition(policeForce);
		Blockade clearBlockade = null;
		double minDistance = (double) Integer.MAX_VALUE;
		if (positionEntity instanceof Road) {
			Road road = (Road) positionEntity;
			if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
				for (Blockade blockade : worldInfo.getBlockades(road)) {
					if (blockade != null) {
						double blockadeDistance = this.getBlockadeDistance(policeForce, blockade);
						if (minDistance > blockadeDistance) {// ハマっている瓦礫を探索
							minDistance = blockadeDistance;
							clearBlockade = blockade;
						}
					}
				}
				if ((int) minDistance < this.clearDistance / 2) {
					// System.out.printf("ext Clear \n");
					pointXY clearPoint = new pointXY();
					clearPoint = this.getBlockadeScaledVector(policeForce, clearBlockade);
					return new ActionClear(clearPoint.X, clearPoint.Y, clearBlockade);
				} else {
					history.setInterruptTargetBlockade(clearBlockade);
					return this.getMovePath(clearBlockade);
				}
				/*  */
			} else {
				// System.out.printf("ext blockade = 0 \n");
				history.resetHistory();
				return null;
			}
		}
		return null;
	}

	

	private Action getRescueAction(PoliceForce police, Road road) {
		double minDistance = Double.MAX_VALUE; // 最も近い人間とのキョリ
		Action moveAction = null;
		Action actionClear = null;
		Collection<StandardEntity> agents = this.worldInfo.getEntitiesOfType(StandardEntityURN.AMBULANCE_TEAM,
				StandardEntityURN.FIRE_BRIGADE);

		for (StandardEntity entity : agents) {
			Human human = (Human) entity;
			if (!human.isPositionDefined() || human.getPosition().getValue() != road.getID().getValue()) {
				continue;
			}

			// Agentとのキョリが最小なら, その救出を仮に戻り値とする
			double distance = this.getDistance(police.getX(), police.getY(), human.getX(), human.getY());
			if (distance < minDistance) {
				minDistance = distance;
				moveAction = new ActionMove(Lists.newArrayList(road.getID()), human.getX(), human.getY());
			}

			// 現在タスクがあれば, それを優先してAction決定
			actionClear = setActionToPerson(human, police, road);
			if (actionClear != null) {
				return actionClear;
			}
		}
		return moveAction;
	}

	/**
	 **/

	private Action setActionToPerson(Human human, PoliceForce police, Road road) {
		//  : 
		this.pathPlanning.setFrom(this.agentInfo.getPosition());
		this.pathPlanning.setDestination(human.getID());
		List<EntityID> path = this.pathPlanning.calc().getResult();
		if (path == null){
			return null;
		} else if (human == null || !human.isPositionDefined()) {
			return new ActionMove(path);
		}

		double humanX = human.getX();
		double humanY = human.getY();
		double policeX = police.getX();
		double policeY = police.getY();
		ActionClear actionClear = null; // 
		Collection<Blockade> blockades = this.worldInfo.getBlockades(road).stream().filter(Blockade::isApexesDefined)
				.collect(Collectors.toSet());

		for (Blockade blockade : blockades) {
			if (!this.isInside(humanX, humanY, blockade.getApexes())) { // 
				continue;
			}
			if (this.intersect(policeX, policeY, humanX, humanY, road)) { // me
				Action action = this.getIntersectEdgeAction(policeX, policeY, humanX, humanY, road);
				if (action != null) { // Avoid error
					if (action.getClass() == ActionClear.class) { // 
						if (actionClear == null) {
							actionClear = (ActionClear) action;
							continue;
						}
						if (actionClear.getTarget() != null) {
							Blockade another = (Blockade) this.worldInfo.getEntity(actionClear.getTarget());
							if (another != null && this.intersect(blockade, another)) {
								return new ActionClear(another);
							}
							int anotherDistance = this.worldInfo.getDistance(police, another);
							int blockadeDistance = this.worldInfo.getDistance(police, blockade);
							if (anotherDistance > blockadeDistance) {
								return action;
							}
						}
						return actionClear;
					}
				}
			} else if (this.intersect(policeX, policeY, humanX, humanY, blockade)) { // me
				Vector2D vector = this.scaleClear(this.getVector(policeX, policeY, humanX, humanY));
				int clearX = (int) (policeX + vector.getX());
				int clearY = (int) (policeY + vector.getY());
				vector = this.scaleBackClear(vector);
				int startX = (int) (policeX + vector.getX());
				int startY = (int) (policeY + vector.getY());
				if (this.intersect(startX, startY, clearX, clearY, blockade)) { // Agent
					if (actionClear == null) {
						actionClear = new ActionClear(clearX, clearY, blockade);
					} else if (actionClear.getTarget() != null) { //
						Blockade another = (Blockade) this.worldInfo.getEntity(actionClear.getTarget());
						if (another != null && this.intersect(blockade, another)) {
							return new ActionClear(another);
						}
						int distance1 = this.worldInfo.getDistance(police, another);
						int distance2 = this.worldInfo.getDistance(police, blockade);
						if (distance1 > distance2) {
							return new ActionClear(clearX, clearY, blockade);
						}
					}
					return actionClear;
				}
			}
		}
		return actionClear; // null
	}

	private Action getAreaClearAction(PoliceForce police, StandardEntity targetEntity) {
		if (targetEntity instanceof Building) {
			return null;
		}
		Road road = (Road) targetEntity;
		if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) {
			return null;
		}
		Collection<Blockade> blockades = this.worldInfo.getBlockades(road).stream().filter(Blockade::isApexesDefined)
				.collect(Collectors.toSet());
		int minDistance = Integer.MAX_VALUE;
		Blockade clearBlockade = null;
		for (Blockade blockade : blockades) {
			for (Blockade another : blockades) {
				if (!blockade.getID().equals(another.getID()) && this.intersect(blockade, another)) {
					int distance1 = this.worldInfo.getDistance(police, blockade);
					int distance2 = this.worldInfo.getDistance(police, another);
					if (distance1 <= distance2 && distance1 < minDistance) {
						minDistance = distance1;
						clearBlockade = blockade;
					} else if (distance2 < minDistance) {
						minDistance = distance2;
						clearBlockade = another;
					}
				}
			}
		}
		if (clearBlockade != null) {
			if (minDistance < this.clearDistance) {
				return new ActionClear(clearBlockade);
			} else {
				history.setInterruptTargetBlockade(clearBlockade);
				return new ActionMove(Lists.newArrayList(police.getPosition()), clearBlockade.getX(),
						clearBlockade.getY());
			}
		}
		double agentX = police.getX();
		double agentY = police.getY();
		clearBlockade = null;
		Double minPointDistance = Double.MAX_VALUE;
		int clearX = 0;
		int clearY = 0;
		for (Blockade blockade : blockades) {
			int[] apexes = blockade.getApexes();
			for (int i = 0; i < (apexes.length - 2); i += 2) {
				double distance = this.getDistance(agentX, agentY, apexes[i], apexes[i + 1]);
				if (distance < minPointDistance) {
					clearBlockade = blockade;
					minPointDistance = distance;
					clearX = apexes[i];
					clearY = apexes[i + 1];
				}
			}
		}
		if (clearBlockade != null) {
			if (minPointDistance < this.clearDistance / 2) {
				Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, clearX, clearY));
				clearX = (int) (agentX + vector.getX());
				clearY = (int) (agentY + vector.getY());
				return new ActionClear(clearX, clearY, clearBlockade);
			}
			history.setInterruptTargetBlockade(clearBlockade);
			return new ActionMove(Lists.newArrayList(police.getPosition()), clearX, clearY);
		}
		return null;
	}

	private Action getNeighbourPositionAction(PoliceForce police, Area target) {
		double agentX = police.getX();
		double agentY = police.getY();
		StandardEntity position = Objects.requireNonNull(this.worldInfo.getPosition(police));
		Edge edge = target.getEdgeTo(position.getID());
		if (edge == null) {
			return null;
		}
		if (position instanceof Road) {
			Road road = (Road) position;
			if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {// 
				double midX = (edge.getStartX() + edge.getEndX()) * 0.5;
				double midY = (edge.getStartY() + edge.getEndY()) * 0.5;
				if (this.intersect(agentX, agentY, midX, midY, road)) {
					// return this.getIntersectEdgeAction(agentX, agentY, edge, road);
				}
				ActionClear actionClear = null;
				ActionMove actionMove = null;
				Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, midX, midY));
				int clearX = (int) (agentX + vector.getX());
				int clearY = (int) (agentY + vector.getY());
				vector = this.scaleBackClear(vector);
				int startX = (int) (agentX + vector.getX());
				int startY = (int) (agentY + vector.getY());
				for (Blockade blockade : this.worldInfo.getBlockades(road)) {
					if (blockade == null || !blockade.isApexesDefined()) {
						continue;
					}
					if (this.intersect(startX, startY, midX, midY, blockade)) {
						if (this.intersect(startX, startY, clearX, clearY, blockade)) {
							if (actionClear == null) {
								// System.out.println("getNeighbourPositionAction 529 ac");
								actionClear = new ActionClear(clearX, clearY, blockade);
								if (this.equalsPoint(this.oldClearX, this.oldClearY, clearX, clearY)) {
									if (this.count >= this.forcedMove) {
										this.count = 0;
										return new ActionMove(Lists.newArrayList(road.getID()), clearX, clearY);

									}
									this.count++;
								}
								this.oldClearX = clearX;
								this.oldClearY = clearY;
							} else {
								if (actionClear.getTarget() != null) {
									Blockade another = (Blockade) this.worldInfo.getEntity(actionClear.getTarget());
									if (another != null && this.intersect(blockade, another)) {
										// System.out.println("getNeighbourPositionAction 544 ac");
										return new ActionClear(another);
									}
								}
								return actionClear;
							}
						} else if (actionMove == null) {
							actionMove = new ActionMove(Lists.newArrayList(road.getID()), (int) midX, (int) midY);
						}
					}
				}
				if (actionClear != null) {
					return actionClear;
				} else if (actionMove != null) {
					return actionMove;
				}
			}
		}
		if (target instanceof Road) {
			Road road = (Road) target;
			if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) {
				return new ActionMove(Lists.newArrayList(position.getID(), target.getID()));
			}
			Blockade clearBlockade = null;
			Double minPointDistance = Double.MAX_VALUE;
			int clearX = 0;
			int clearY = 0;
			int[] temp = null;
			for (EntityID id : road.getBlockades()) {
				Blockade blockade = (Blockade) this.worldInfo.getEntity(id);
				if (blockade != null && blockade.isApexesDefined()) {
					int[] apexes = blockade.getApexes();
					for (int i = 0; i < (apexes.length - 2); i += 2) {
						double distance = this.getDistance(agentX, agentY, apexes[i], apexes[i + 1]);

						if (distance < minPointDistance) {
							clearBlockade = blockade;
							minPointDistance = distance;
							clearX = apexes[i];
							clearY = apexes[i + 1];
						}
					}
				}
			}
			if (clearBlockade != null && minPointDistance < this.clearDistance / 2) {
				Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, clearX, clearY));
				clearX = (int) (agentX + vector.getX());
				clearY = (int) (agentY + vector.getY());
				if (this.equalsPoint(this.oldClearX, this.oldClearY, clearX, clearY)) {
					if (this.count >= this.forcedMove) {
						this.count = 0;
						return new ActionMove(Lists.newArrayList(road.getID()), clearX, clearY);
					}
					this.count++;
				}
				this.oldClearX = clearX;
				this.oldClearY = clearY;
				// System.out.println("getNeighbourPositionAction 599 ac");
				return new ActionClear(clearX, clearY, clearBlockade);
			}
		}
		return new ActionMove(Lists.newArrayList(position.getID(), target.getID()));
	}

	private Action getIntersectEdgeAction(double agentX, double agentY, Edge edge, Road road) {
		double midX = (edge.getStartX() + edge.getEndX()) / 2;
		double midY = (edge.getStartY() + edge.getEndY()) / 2;
		return this.getIntersectEdgeAction(agentX, agentY, midX, midY, road);
	}

	private Action getIntersectEdgeAction(double agentX, double agentY, double pointX, double pointY, Road road) {
		Set<Point2D> movePoints = this.getMovePoints(road);
		Point2D bestPoint = null;
		double bastDistance = Double.MAX_VALUE;
		for (Point2D p : movePoints) {
			if (!this.intersect(agentX, agentY, p.getX(), p.getY(), road)) {
				if (!this.intersect(pointX, pointY, p.getX(), p.getY(), road)) {
					double distance = this.getDistance(pointX, pointY, p.getX(), p.getY());
					if (distance < bastDistance) {
						bestPoint = p;
						bastDistance = distance;
					}
				}
			}
		}
		if (bestPoint != null) {
			double pX = bestPoint.getX();
			double pY = bestPoint.getY();
			if (!road.isBlockadesDefined()) {
				return new ActionMove(Lists.newArrayList(road.getID()), (int) pX, (int) pY);
			}
			ActionClear actionClear = null;
			ActionMove actionMove = null;
			Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, pX, pY));
			int clearX = (int) (agentX + vector.getX());
			int clearY = (int) (agentY + vector.getY());
			vector = this.scaleBackClear(vector);
			int startX = (int) (agentX + vector.getX());
			int startY = (int) (agentY + vector.getY());
			for (Blockade blockade : this.worldInfo.getBlockades(road)) {
				if (this.intersect(startX, startY, pX, pY, blockade)) {
					if (this.intersect(startX, startY, clearX, clearY, blockade)) {
						if (actionClear == null) {
							actionClear = new ActionClear(clearX, clearY, blockade);
						} else {
							if (actionClear.getTarget() != null) {
								Blockade another = (Blockade) this.worldInfo.getEntity(actionClear.getTarget());
								if (another != null && this.intersect(blockade, another)) {
									return new ActionClear(another);
								}
							}
							return actionClear;
						}
					} else if (actionMove == null) {
						actionMove = new ActionMove(Lists.newArrayList(road.getID()), (int) pX, (int) pY);
					}
				}
			}
			if (actionClear != null) {
				return actionClear;
			} else if (actionMove != null) {
				return actionMove;
			}
		}
		Action action = this.getAreaClearAction((PoliceForce) this.agentInfo.me(), road);
		if (action == null) {
			action = new ActionMove(Lists.newArrayList(road.getID()), (int) pointX, (int) pointY);
		}
		return action;
	}

	private boolean equalsPoint(double p1X, double p1Y, double p2X, double p2Y) {
		return this.equalsPoint(p1X, p1Y, p2X, p2Y, 1000.0D);
	}

	private boolean equalsPoint(double p1X, double p1Y, double p2X, double p2Y, double range) {
		return (p2X - range < p1X && p1X < p2X + range) && (p2Y - range < p1Y && p1Y < p2Y + range);
	}

	private boolean isInside(double pX, double pY, int[] apex) {
		Point2D p = new Point2D(pX, pY);
		Vector2D v1 = (new Point2D(apex[apex.length - 2], apex[apex.length - 1])).minus(p);
		Vector2D v2 = (new Point2D(apex[0], apex[1])).minus(p);
		double theta = this.getAngle(v1, v2);

		for (int i = 0; i < apex.length - 2; i += 2) {
			v1 = (new Point2D(apex[i], apex[i + 1])).minus(p);
			v2 = (new Point2D(apex[i + 2], apex[i + 3])).minus(p);
			theta += this.getAngle(v1, v2);
		}
		return Math.round(Math.abs((theta / 2) / Math.PI)) >= 1;
	}

	private boolean intersect(double agentX, double agentY, double pointX, double pointY, Area area) {
		for (Edge edge : area.getEdges()) {
			double startX = edge.getStartX();
			double startY = edge.getStartY();
			double endX = edge.getEndX();
			double endY = edge.getEndY();
			if (java.awt.geom.Line2D.linesIntersect(agentX, agentY, pointX, pointY, startX, startY, endX, endY)) {
				double midX = (edge.getStartX() + edge.getEndX()) / 2;
				double midY = (edge.getStartY() + edge.getEndY()) / 2;
				if (!equalsPoint(pointX, pointY, midX, midY) && !equalsPoint(agentX, agentY, midX, midY)) {
					return true;
				}
			}
		}
		return false;
	}

	private boolean intersect(Blockade blockade, Blockade another) {
		if (blockade.isApexesDefined() && another.isApexesDefined()) {
			int[] apexes0 = blockade.getApexes();
			int[] apexes1 = another.getApexes();
			for (int i = 0; i < (apexes0.length - 2); i += 2) {
				for (int j = 0; j < (apexes1.length - 2); j += 2) {
					if (java.awt.geom.Line2D.linesIntersect(apexes0[i], apexes0[i + 1], apexes0[i + 2], apexes0[i + 3],
							apexes1[j], apexes1[j + 1], apexes1[j + 2], apexes1[j + 3])) {
						return true;
					}
				}
			}
			for (int i = 0; i < (apexes0.length - 2); i += 2) {
				if (java.awt.geom.Line2D.linesIntersect(apexes0[i], apexes0[i + 1], apexes0[i + 2], apexes0[i + 3],
						apexes1[apexes1.length - 2], apexes1[apexes1.length - 1], apexes1[0], apexes1[1])) {
					return true;
				}
			}
			for (int j = 0; j < (apexes1.length - 2); j += 2) {
				if (java.awt.geom.Line2D.linesIntersect(apexes0[apexes0.length - 2], apexes0[apexes0.length - 1],
						apexes0[0], apexes0[1], apexes1[j], apexes1[j + 1], apexes1[j + 2], apexes1[j + 3])) {
					return true;
				}
			}
		}
		return false;
	}

	private boolean intersect(double agentX, double agentY, double pointX, double pointY, Blockade blockade) {
		List<Line2D> lines = GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(blockade.getApexes()),
				true);
		for (Line2D line : lines) {
			Point2D start = line.getOrigin();
			Point2D end = line.getEndPoint();
			double startX = start.getX();
			double startY = start.getY();
			double endX = end.getX();
			double endY = end.getY();
			if (java.awt.geom.Line2D.linesIntersect(agentX, agentY, pointX, pointY, startX, startY, endX, endY)) {
				return true;
			}
		}
		return false;
	}

	/**
	 * actionClear対象の瓦礫へのベクトルを調整して返す。瓦礫をそのまま対象とした際に使用される座標が不明なため作成。
	 *
	 * @param policeForce
	 * @param blockade
	 * @return pointXY(int X, int Y)対象のXY座標
	 */
	private pointXY getBlockadeScaledVector(PoliceForce police, Blockade blockade) {
		Double minPointDistance = Double.MAX_VALUE;
		pointXY clearPoint = new pointXY();
		clearPoint.X = clearPoint.Y = 0;
		double agentX = police.getX();
		double agentY = police.getY();
		if (blockade != null && blockade.isApexesDefined()) {
			int[] apexes = blockade.getApexes();
			for (int i = 0; i < (apexes.length - 2); i += 2) {
				double distance = this.getDistance(agentX, agentY, apexes[i], apexes[i + 1]);
				if (distance < minPointDistance) {
					minPointDistance = distance;
					clearPoint.X = apexes[i];
					clearPoint.Y = apexes[i + 1];
				}
			}
		}
		Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, clearPoint.X, clearPoint.Y));
		clearPoint.X = (int) (agentX + vector.getX());
		clearPoint.Y = (int) (agentY + vector.getY());
		return clearPoint;
	}

	/**
	 * 
	 *
	 * @param police
	 * @param blockade
	 * @return 
	 */
	private double getBlockadeDistance(PoliceForce police, Blockade blockade) {
		double agentX = police.getX();
		double agentY = police.getY();
		int[] apexes = blockade.getApexes();
		double minPointDistance = Double.MAX_VALUE;
		for (int i = 0; i < (apexes.length - 2); i += 2) {
			double distance = this.getDistance(agentX, agentY, apexes[i], apexes[i + 1]);
			if (distance < minPointDistance) {
				minPointDistance = distance;
			}
		}
		return minPointDistance;
	}

	private double getDistance(double fromX, double fromY, double toX, double toY) {
		double dx = toX - fromX;
		double dy = toY - fromY;
		return Math.hypot(dx, dy);
	}

	private double getAngle(Vector2D v1, Vector2D v2) {
		double flag = (v1.getX() * v2.getY()) - (v1.getY() * v2.getX());
		double angle = Math
				.acos(((v1.getX() * v2.getX()) + (v1.getY() * v2.getY())) / (v1.getLength() * v2.getLength()));
		if (flag > 0) {
			return angle;
		}
		if (flag < 0) {
			return -1 * angle;
		}
		return 0.0D;
	}

	private Vector2D getVector(double fromX, double fromY, double toX, double toY) {
		return (new Point2D(toX, toY)).minus(new Point2D(fromX, fromY));
	}

	private Vector2D scaleClear(Vector2D vector) {
		return vector.normalised().scale(this.clearDistance);
	}

	private Vector2D scaleBackClear(Vector2D vector) {
		return vector.normalised().scale(-510);
	}

	private Set<Point2D> getMovePoints(Road road) {
		Set<Point2D> points = this.movePointCache.get(road.getID());
		if (points == null) {
			points = new HashSet<>();
			int[] apex = road.getApexList();
			for (int i = 0; i < apex.length; i += 2) {
				for (int j = i + 2; j < apex.length; j += 2) {
					double midX = (apex[i] + apex[j]) / 2;
					double midY = (apex[i + 1] + apex[j + 1]) / 2;
					if (this.isInside(midX, midY, apex)) {
						points.add(new Point2D(midX, midY));
					}
				}
			}
			for (Edge edge : road.getEdges()) {
				double midX = (edge.getStartX() + edge.getEndX()) / 2;
				double midY = (edge.getStartY() + edge.getEndY()) / 2;
				points.remove(new Point2D(midX, midY));
			}
			this.movePointCache.put(road.getID(), points);
		}
		return points;
	}

	private boolean needRest(Human agent) {
		int hp = agent.getHP();
		int damage = agent.getDamage();
		if (damage == 0 || hp == 0) {
			return false;
		}
		int activeTime = (hp / damage) + ((hp % damage) != 0 ? 1 : 0);
		if (this.kernelTime == -1) {
			try {
				this.kernelTime = this.scenarioInfo.getKernelTimesteps();
			} catch (NoSuchConfigOptionException e) {
				this.kernelTime = -1;
			}
		}
		return damage >= this.thresholdRest || (activeTime + this.agentInfo.getTime()) < this.kernelTime;
	}

	private Action calcRest(Human human, PathPlanning pathPlanning, Collection<EntityID> targets) {
		EntityID position = human.getPosition();
		Collection<EntityID> refuges = this.worldInfo.getEntityIDsOfType(StandardEntityURN.REFUGE);
		int currentSize = refuges.size();
		if (refuges.contains(position)) {
			return new ActionRest();
		}
		List<EntityID> firstResult = null;
		while (refuges.size() > 0) {
			pathPlanning.setFrom(position);
			pathPlanning.setDestination(refuges);
			List<EntityID> path = pathPlanning.calc().getResult();
			if (path != null && path.size() > 0) {
				if (firstResult == null) {
					firstResult = new ArrayList<>(path);
					if (targets == null || targets.isEmpty()) {
						break;
					}
				}
				EntityID refugeID = path.get(path.size() - 1);
				pathPlanning.setFrom(refugeID);
				pathPlanning.setDestination(targets);
				List<EntityID> fromRefugeToTarget = pathPlanning.calc().getResult();
				if (fromRefugeToTarget != null && fromRefugeToTarget.size() > 0) {
					return new ActionMove(path);
				}
				refuges.remove(refugeID);
				// remove failed
				if (currentSize == refuges.size()) {
					break;
				}
				currentSize = refuges.size();
			} else {
				break;
			}
		}
		return firstResult != null ? new ActionMove(firstResult) : null;
	}

	/**
	 * 指定したHumanが瓦礫に挟まっているならそのBlockadeを返す関数
	 *
	 * @param human 指定するhuman
	 * @return blockade or null
	 * @author Okajima(13rd)
	 */
	private Blockade blockadeWithHuman(Human human) {
		if (!human.isXDefined() || !human.isYDefined())
			return null;
		int agentX = human.getX();
		int agentY = human.getY();
		StandardEntity positionEntity = this.worldInfo.getPosition(human);
		if (positionEntity instanceof Road) {
			Road road = (Road) positionEntity;
			if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
				for (Blockade blockade : worldInfo.getBlockades(road)) {
					if (blockade.getShape().contains(agentX, agentY)) {
						return blockade;
					}
				}
			}
		}
		return null;
	}

	/**
	 * 指定したHumanが拡張した瓦礫に挟まっているならtrueを返す関数
	 *
	 * @return true or false
	 * @author Okajima(13rd)
	 * @author Amada(14th)
	 */
	private boolean isInExtendedBlockade(Human human) {
		if (!human.isXDefined() || !human.isYDefined())
			return false;
		int agentX = human.getX();
		int agentY = human.getY();
		StandardEntity positionEntity = this.worldInfo.getPosition(human);
		if (positionEntity instanceof Road) {
			Road road = (Road) positionEntity;
			if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
				for (Blockade blockade : worldInfo.getBlockades(road)) {
					Shape extendedShape = getExtendedShape(blockade);
					if (extendedShape != null && extendedShape.contains(agentX, agentY)) {
						return true;
					}
				}
			}
		}
		return false;
	}

	/**
	 * 重心方向から各頂点方向に2だけ広げたShapeを返す.瓦礫に引っかかりにくくするための関数．
	 *
	 * @param blockade blockade
	 * @return 重心方向から各頂点方向に2だけ広げたShapeを返す
	 * @author Okajima(13rd)
	 */

	private Shape getExtendedShape(Blockade blockade) {
		Shape shape = null;
		int[] allApexes = blockade.getApexes();
		int count = allApexes.length / 2;
		int[] xs = new int[count];
		int[] ys = new int[count];
		double centerX = 0;
		double centerY = 0;
		for (int i = 0; i < count; ++i) {
			xs[i] = allApexes[i * 2];
			ys[i] = allApexes[i * 2 + 1];
			centerX += xs[i];
			centerY += ys[i];
		}
		centerX /= count;
		centerY /= count;
		for (int i = 0; i < count; ++i) {
			// 重心から頂点へのベクトル
			double vectorX = xs[i] - centerX;
			double vectorY = ys[i] - centerY;
			double magnitude = Math.sqrt(vectorX * vectorX + vectorY * vectorY); // ベクトルの大きさ
			// 重心から頂点への大きさ2のベクトルを頂点に足して四捨五入
			xs[i] += (vectorX / magnitude) * 2 + 0.5;
			ys[i] += (vectorY / magnitude) * 2 + 0.5;
		}
		shape = new Polygon(xs, ys, count);
		return shape;
	}

	private boolean isStopping() {
        Pair<Integer,Integer> location = this.worldInfo.getLocation(this.agentInfo.me().getID());
        Pair<Integer, Integer> previousLocation = this.worldInfo.getLocation(-1, this.agentInfo.me().getID());
        boolean isStopping = false;
        int threadHoldReach = 20;

        if (!(this.agentInfo.getExecutedAction(-1) instanceof ActionMove)) {
            isStopping = false;
        }else if (this.worldInfo.getLocation(this.worldInfo.getPosition(-1, this.agentInfo.me().getID())) == this.worldInfo.getLocation(this.agentInfo.getPosition())) {
            isStopping = true;
        }
        else if (Math.pow(location.first() - previousLocation.first(), 2) + Math.pow(location.second() - previousLocation.second(), 2) < Math.pow(threadHoldReach, 2)) {
          isStopping = true;
        }
        return isStopping;
    }

	private Shape getExtendedShape(Road road, int extend) {
		Shape shape = null;
		int[] allApexes = road.getApexList();
		int count = allApexes.length / 2;
		int[] xs = new int[count];
		int[] ys = new int[count];
		double centerX = 0;
		double centerY = 0;
		for (int i = 0; i < count; ++i) {
			xs[i] = allApexes[i * 2];
			ys[i] = allApexes[i * 2 + 1];
			centerX += xs[i];
			centerY += ys[i];
		}
		centerX /= count;
		centerY /= count;
		for (int i = 0; i < count; ++i) {
			// 重心から頂点へのベクトル
			double vectorX = xs[i] - centerX;
			double vectorY = ys[i] - centerY;
			double magnitude = Math.sqrt(vectorX * vectorX + vectorY * vectorY); // ベクトルの大きさ
			// 重心から頂点への大きさ2のベクトルを頂点に足して四捨五入
			xs[i] += (vectorX / magnitude) * extend + 0.5;
			ys[i] += (vectorY / magnitude) * extend + 0.5;
		}
		shape = new Polygon(xs, ys, count);
		return shape;
	}

	/****
	 * @author : Horie（16th） Humanを再ターゲットする checkDeadEndと対を成す関数になる予定
	 ****/
	private Action removeDeadEnd() {
		// 0.fDEが扱うべきでない場合はパス（nullを返す）->ココに書くべきか？
		if (!(this.worldInfo.getEntity(this.target) instanceof Road) || this.target == null) {
			return null;
		}

		boolean isBuried = false;
		boolean isSuffered;
		double nearDistance;
		double distance;

		final Vector2D vecOrigin = new Vector2D(0, 1);
		final StandardEntity myself = this.agentInfo.me();
		final Human myHuman = (Human) myself;
		final List<EntityID> allHuman = new ArrayList<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.CIVILIAN,
				StandardEntityURN.AMBULANCE_TEAM, StandardEntityURN.FIRE_BRIGADE));

		Action resultAction = null;
		Blockade nearBlock = null;
		Human nearest = null;
		Vector2D targetVec;
		List<EntityID> path;
		List<EntityID> neighbourRoads;
		List<EntityID> suffers = new ArrayList<>(); // 「立ち往生」している人
		List<EntityID> allBlockades = new ArrayList<>();

		// 1.下準備：targetかそのneighbourにがれきがない場合はパス
		neighbourRoads = ((Area) (this.worldInfo.getEntity(this.target))).getNeighbours();
		neighbourRoads.add(this.target);
		Collection<Blockade> tmpBList = this.worldInfo.getBlockades(this.target);
		for (Blockade tmpBlockade : tmpBList) {
			allBlockades.add(tmpBlockade.getID()); // 瓦礫を追加
		}

		for (EntityID tmpNeighborId : neighbourRoads) { // target及び周りの道路についてチェック
			StandardEntity tmpNeighbor = this.worldInfo.getEntity(tmpNeighborId);
			if (!(tmpNeighbor instanceof Road)) {
				continue;
			}
			if ((((Road) tmpNeighbor).isBlockadesDefined() && ((Road) tmpNeighbor).getBlockades().size() > 0)) {
				isBuried = true;
				break;
			}

			tmpBList = this.worldInfo.getBlockades(tmpNeighborId);
			for (Blockade tmpBlockade : tmpBList){
				allBlockades.add(tmpBlockade.getID()); // 瓦礫を追加
			}
		}
		if (!isBuried) {
			return null;
		}

		// 2.target及び周辺の道にいる人をリストアップ
		for (EntityID tmpHumanID : allHuman) {
			for (EntityID tmpNeighborId : neighbourRoads) {
				StandardEntity tmpHuman = worldInfo.getEntity(tmpHumanID);
				// tmpRoad = worldInfo.getEntity(tmpNeighborId);
				isSuffered = (((Human) tmpHuman).getPosition().equals(tmpNeighborId));
				if (isSuffered) {
					suffers.add(tmpHumanID);
				}
			}
		}
		if (suffers.isEmpty()) { // suffersが空なら
			return null;
		}

		// 3.人が密集しているとこを調べ, とりあえず目標となる人間を決める(要検討)
		nearDistance = Double.MAX_VALUE;
		for (EntityID tmpHumanID : suffers) {
			StandardEntity tmpHuman = worldInfo.getEntity(tmpHumanID);
			distance = this.worldInfo.getDistance(tmpHuman, myself);
			if (nearDistance > distance) {
				nearDistance = distance;
				nearest = (Human) tmpHuman;
			}
		}
		if (nearest == null){
			return null;
		}

		// 4.目標の近くの瓦礫を求める
		nearDistance = Double.MAX_VALUE;
		for (EntityID tmpBlockID : allBlockades) {
			StandardEntity tmpBlock = worldInfo.getEntity(tmpBlockID);
			distance = this.worldInfo.getDistance(tmpBlock, nearest);
			if (((Blockade) tmpBlock).isPositionDefined() && nearDistance > distance
					&& (Blockade) tmpBlock != formerTarget) {
				nearDistance = distance;
				nearBlock = (Blockade) tmpBlock;
			}
		}
		if (nearBlock == null) {
			return null;
		}

		// 5.最後に行動を決定
		// blockadeを消せる範囲外の場合
		if (this.worldInfo.getDistance(nearBlock, myself) > this.clearDistance / 2.0 - this.agentRadius) { // 目標が遠すぎる
			return setActionToPerson(
				nearest, (PoliceForce) myself,
				(Road) (this.worldInfo.getEntity(this.target))
			);
		}

		// 消せる範囲内の場合
		targetVec = new Vector2D(nearBlock.getX() - myHuman.getX(), nearBlock.getY() - myHuman.getY());
		if (targetVec.getLength() == 0) { // 0ベクトルを回避
			targetVec = vecOrigin;
		}
		/*
			* if(formerTarget == nearBlock){ //重心付近が除去済みのときはややずらす this.random = new
			* Random(); theta = getAngle(vecOrigin, targetVec) + (random.nextDouble() -
			* 0.5) * Math.PI; targetVec = new Vector2D(Math.cos(theta),Math.sin(theta));
			* //[Debug Code] System.out.print("Re:"); }
			*/
		targetVec = targetVec.normalised().scale(this.clearDistance);
		resultAction = new ActionClear(this.agentInfo, targetVec);
		formerTarget = nearBlock;
		return resultAction;
	}

	// 目標までのパスプラ処理を行い、actionMoveを返す
	private Action getMovePath(Blockade destination) {
		PoliceForce me = (PoliceForce) this.agentInfo.me();
		this.pathPlanning.setFrom(this.agentInfo.getPosition());
		this.pathPlanning.setDestination(destination.getID());
		List<EntityID> path;
		path = this.pathPlanning.calc().getResult();
		Action result = null;
		if (path != null) {
			result = new ActionMove(path);
		} else {
			if (!(this.worldInfo.getEntity(this.target) instanceof Road) || this.target == null) {
				result = null;
			}
			else {
				result = setActionToPerson((Human) this.agentInfo.me(), (PoliceForce) me,
						(Road) (this.worldInfo.getEntity(this.target)));
			}
		}
		return result;
	}

    /**
	 * @author Nomoto(17th)
	 * @since 2023 集団になりすぎて探索や除去の効率が悪くなることを防ぐために
	 * 2人以上の集団になった場合EntityIDが若い2人を1グループとしたい
	 * そのために、2人以上の集団になったとき自分のEntityIDが2番目以降ならTargetを変更させる
	 **/
	private EntityID makeTwoManCell(EntityID target){
		EntityID myPotsitionID = this.agentInfo.getPosition();
		List<Integer> nearPoriceForce = new ArrayList<>();
		nearPoriceForce.add(myPotsitionID.getValue());//(自分を含めた)近くのPF
		EntityID anotherTarget = null;

		for (EntityID policeForceID : this.worldInfo.getEntityIDsOfType(StandardEntityURN.POLICE_FORCE)) {
			//他のPFとの距離が500未満の場合、密接していると考える(500はエージェントの半径)
			if(this.worldInfo.getDistance(myPotsitionID, policeForceID) < 500){
				nearPoriceForce.add(policeForceID.getValue());
			}
		}
		Collections.sort(nearPoriceForce);//EntityID順にソート
		//(自分を含めた)近くのPFが2以上のときEntityIDによって判定
		if(nearPoriceForce.size() <= 2){
			return target;
		}
		else if(!(nearPoriceForce.get(1) < myPotsitionID.getValue())){
			return target;
		}
		else{
			//自身の現在地にあるガレキ
			//List<EntityID> targetBlockades = ((Road)this.worldInfo.getEntity(myPotsitionID)).getBlockades();
			// while(true){
			// 	int index = random.nextInt(targetBlockades.size());
			// 	anotherTarget = targetBlockades(index);
			// 	if(!(anotherTarget = target)){
			// 		break
			// 	}
			// }
			// return anotherTarget
			return null;
		}
	}
}