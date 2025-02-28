package TIMRAD_2025.extaction;

import adf.core.agent.action.Action;
import adf.core.agent.action.ambulance.ActionRescue;
import adf.core.agent.action.common.ActionMove;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.core.agent.communication.standard.bundle.StandardMessagePriority;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.extaction.ExtAction;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import adf.core.component.extaction.ExtAction;
import adf.core.agent.communication.standard.bundle.centralized.*;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import adf.core.component.module.complex.Search;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;
import adf.core.agent.action.Action;
import adf.core.agent.action.common.ActionMove;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.StandardCommunicationModule;
import adf.core.component.communication.CommunicationMessage;
import adf.core.agent.communication.standard.bundle.information.*;
import adf.core.agent.communication.standard.bundle.MessageUtil;
import adf.core.agent.communication.standard.bundle.StandardMessage;


import java.util.*;

//import javax.swing.text.html.parser.Entity;

import rescuecore2.config.NoSuchConfigOptionException;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import static rescuecore2.standard.entities.StandardEntityURN.BLOCKADE;
import static rescuecore2.standard.entities.StandardEntityURN.BUILDING;
import static rescuecore2.standard.entities.StandardEntityURN.REFUGE;

public class TIMRADActionFireRescue extends ExtAction {

	private PathPlanning pathPlanning;

	private Clustering clustering;
	private boolean stopped;
	private ArrayList<EntityID> unsearchedBuildingIDs;
	private int clusterIndex;
	private int changeClusterCycle;
	protected Random random;
	private ArrayList<Point2D> previousLocations;
	private ArrayList<List<EntityID>> previousPaths;

	private int          thresholdRest;
	private int          kernelTime;

	private EntityID     target;

	private boolean isRescued;
	private Human targetingHuman;


	public TIMRADActionFireRescue( AgentInfo agentInfo, WorldInfo worldInfo, ScenarioInfo scenarioInfo, ModuleManager moduleManager, DevelopData developData ) {
		super( agentInfo, worldInfo, scenarioInfo, moduleManager, developData );
		this.target = null;
		this.thresholdRest = developData.getInteger( "ActionFireRescue.rest", 100 );

		switch ( scenarioInfo.getMode() ) {
		case PRECOMPUTATION_PHASE:
			this.pathPlanning = moduleManager.getModule(
					"ActionFireRescue.PathPlanning",
					"adf.core.component.module.algorithm.PathPlanning" );
			break;
		case PRECOMPUTED:
			this.pathPlanning = moduleManager.getModule(
					"ActionFireRescue.PathPlanning",
					"adf.core.component.module.algorithm.PathPlanning" );
			break;
		case NON_PRECOMPUTE:
			this.pathPlanning = moduleManager.getModule(
					"ActionFireRescue.PathPlanning",
					"adf.core.component.module.algorithm.PathPlanning" );
			break;
		}

		// calcSearch用(これも何個かいらないかも)
		this.clustering = moduleManager.getModule("ActionTransport.Clustering.Ambulance", "RIO2023.algorithm.RioneKmeansPP");
		unsearchedBuildingIDs = new ArrayList<>();
		this.changeClusterCycle = 5;
		this.clusterIndex = 0;
		this.random = new Random();
		this.stopped = false;
		this.previousLocations = new ArrayList<>();
		this.previousPaths = new ArrayList<>();

		this.isRescued = false;
		this.targetingHuman = null;
	}


	public ExtAction precompute( PrecomputeData precomputeData ) {
		super.precompute( precomputeData );
		if ( this.getCountPrecompute() >= 2 ) {
			return this;
		}
		this.pathPlanning.precompute( precomputeData );
		try {
			this.kernelTime = this.scenarioInfo.getKernelTimesteps();
		} catch ( NoSuchConfigOptionException e ) {
			this.kernelTime = -1;
		}
		return this;
	}


	public ExtAction resume( PrecomputeData precomputeData ) {
		super.resume( precomputeData );
		if ( this.getCountResume() >= 2 ) {
			return this;
		}
		this.pathPlanning.resume( precomputeData );
		try {
			this.kernelTime = this.scenarioInfo.getKernelTimesteps();
		} catch ( NoSuchConfigOptionException e ) {
			this.kernelTime = -1;
		}
		return this;
	}


	public ExtAction preparate() {
		super.preparate();
		if ( this.getCountPreparate() >= 2 ) {
			return this;
		}
		this.pathPlanning.preparate();
		try {
			this.kernelTime = this.scenarioInfo.getKernelTimesteps();
		} catch ( NoSuchConfigOptionException e ) {
			this.kernelTime = -1;
		}
		return this;
	}


	public ExtAction updateInfo( MessageManager messageManager ) {
		super.updateInfo( messageManager );
		if ( this.getCountUpdateInfo() >= 2 ) {
			return this;
		}
		this.pathPlanning.updateInfo( messageManager );
		if (this.unsearchedBuildingIDs.isEmpty()) {
			this.reset();
		}

		// 未探索建物の精査
		List<EntityID> perceivedBuildings = new ArrayList<>();// 見つけた建物
		for (EntityID id : worldInfo.getChanged().getChangedEntities()) {
			StandardEntity se = worldInfo.getEntity(id);
			if (se instanceof Building) {
				perceivedBuildings.add(id);
			}
		}
		for (EntityID pID : perceivedBuildings) {
			//not need to check list contains
			unsearchedBuildingIDs.remove(pID);
		}

		return this;
	}


	@Override
	public ExtAction setTarget( EntityID target ) {
		this.target = null;
		if ( target != null ) {
			StandardEntity entity = this.worldInfo.getEntity( target );
			if ( entity instanceof Human || entity instanceof Area ) {
				this.target = target;
				return this;
			}
		}
		return this;
	}


	@Override
	public ExtAction calc() {
		this.result = null;
		FireBrigade agent = (FireBrigade) this.agentInfo.me();
		// System.out.printf("RIOActionFireRescue calc call\n");

		if((this.result = this.continueRescue(agent)) != null){
			return this;
		}

		if ( this.needRest( agent ) ) {
			EntityID areaID = this.convertArea( this.target );
			ArrayList<EntityID> targets = new ArrayList<>();
			if ( areaID != null ) {
				targets.add( areaID );
			}
		}
		if ( this.target != null ) {
			this.result = this.calcRescue( agent, this.pathPlanning, this.target );
		}
		//ダメージを負った市民を見かけたら即座にターゲットを変更し、救助速度を上げる
		Set<EntityID> changedEntityIDs = new HashSet<EntityID>(this.worldInfo.getChanged().getChangedEntities());
		for(EntityID changedEntityID : changedEntityIDs) {
			StandardEntity changedEntity = this.worldInfo.getEntity(changedEntityID);
			if(changedEntity instanceof Civilian){
				this.target = changedEntityID;
			}
		}
		//掘り終わった市民を掘り続ける問題防止
		Human civilian = (Human)this.worldInfo.getEntity(this.target);
		if (civilian.getBuriedness() == 0){
			this.result = null;
		}
		//救助中でなく、ターゲットとの距離が変わらなければせき止められてる可能性が高いのでターゲット変更
		//Human civilian = (Human) Objects.requireNonNull(this.worldInfo.getEntity(this.agentInfo))
		if(!(this.agentInfo.getExecutedAction(-1) instanceof ActionRescue) && this.isStopping()){
			this.result = null;
		}
		return this;
	}

  //対象が停止しているかどうか
    private boolean isStopping() {
        Pair<Integer,Integer> location = this.worldInfo.getLocation(this.agentInfo.me().getID());
        Pair<Integer, Integer> previousLocation = this.worldInfo.getLocation(-1, this.agentInfo.me().getID());
        boolean isStopping = false;
		//停止しているかどうかを判定、1サイクルの移動距離がthreadHoldReach以内なら停止状態と判定
        int threadHoldReach = 1;

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

	//止まってる判定はtrue、止まってなければfalse
	private boolean isStopped(List<EntityID> path1, List<EntityID> path2) {
		Human agent = (Human) this.agentInfo.me();
		previousLocations.add(new Point2D(agent.getX(), agent.getY()));//移動するときの場所を記録(0が現在地)

		if (path1 == null || path2 == null) {
			return false;
		}
		if (path1.size() != path2.size()) {
			return false;
		} else {
			for (int i = 0; i < path1.size(); i++) {
				EntityID id1 = path1.get(i);
				EntityID id2 = path2.get(i);
				if (!id1.equals(id2))
					return false;
			}
		}

		if (previousLocations.size() > 2) {
			return withinRange(previousLocations.get(0), previousLocations.get(1), previousLocations.get(2));
		}
		return false;
	}

	private boolean withinRange(Point2D position1, Point2D position2, Point2D position3) {
		int range = 30000;

		double dist1 = GeometryTools2D.getDistance(position1, position2);
		double dist2 = GeometryTools2D.getDistance(position1, position3);

		return dist1 < range && dist2 < range;

	}

	private void reset() {
		this.unsearchedBuildingIDs.clear();
		this.previousPaths.clear();
		this.previousLocations.clear();

		if ((this.agentInfo.getTime() != 0 && (this.agentInfo.getTime() % this.changeClusterCycle) == 0) || stopped) {
			this.stopped = false;
			this.clusterIndex = random.nextInt(clustering.getClusterNumber());
			this.changeClusterCycle = random.nextInt(16) + 15;//変更

		}
		Collection<StandardEntity> clusterEntities = new ArrayList<>();
		if (clustering != null) {
			clusterEntities.addAll(this.clustering.getClusterEntities(clusterIndex));
		}

		if (!clusterEntities.isEmpty() && clusterEntities.size() > 0) {
			for (StandardEntity entity : clusterEntities) {
				if (entity instanceof Building && entity.getStandardURN() != REFUGE) {
					this.unsearchedBuildingIDs.add(entity.getID());
				}
			}
		} else {
			this.unsearchedBuildingIDs.addAll(this.worldInfo.getEntityIDsOfType(BUILDING));
		}
	}

	private Action calcActionMove(List<EntityID> path) {
        previousPaths.add(path);
        if (previousPaths.size() < 2 || !isStopped(previousPaths.get(0), previousPaths.get(1))) {
            if (path != null && path.size() > 0) {
                StandardEntity entity = this.worldInfo.getEntity(path.get(path.size() - 1));
                if (entity instanceof Building) {
                    if (entity.getStandardURN() != StandardEntityURN.REFUGE && ((Building) entity).isOnFire()) {
                        path.remove(path.size() - 1);
                    }
                }
                if (path.size() > 0) {
                    return new ActionMove(path);
                }
            }
            return null;
        }
        return null;
    }

	// 市民を救助し続けるか？
	private Action continueRescue(FireBrigade FB){
		if(this.isRescued == false || this.targetingHuman == null){
			return null;
		}
		Human human = this.targetingHuman;

		this.isRescued = false;
		this.targetingHuman = null;


		EntityID FB_Position = FB.getPosition();
		if ( !human.isPositionDefined() ) {
			return null;
		}
		if ( human.isHPDefined() && human.getHP() == 0 ) {
			return null;
		}
		EntityID targetPosition = human.getPosition();
		if ( FB_Position.getValue() == targetPosition.getValue() ) {
			if ( human.isBuriednessDefined() && human.getBuriedness() > 0 ) {
				this.targetingHuman = human;
				this.isRescued = true;
				return new ActionRescue( human );
			}
		}
		return null;
	}

	private Action calcRescue( FireBrigade agent, PathPlanning pathPlanning,
			EntityID targetID ) {
		StandardEntity targetEntity = this.worldInfo.getEntity( targetID );
		if ( targetEntity == null ) {
			return null;
		}
		EntityID agentPosition = agent.getPosition();
		if ( targetEntity instanceof Human ) {
			Human human = (Human) targetEntity;
			if ( !human.isPositionDefined() ) {
				return null;
			}
			if ( human.isHPDefined() && human.getHP() == 0 ) {
				return null;
			}
			EntityID targetPosition = human.getPosition();
			if (human.isBuriednessDefined() && human.getBuriedness() > 0){
				if ( agentPosition.getValue() == targetPosition.getValue()) {
					this.isRescued = true;
					this.targetingHuman = human;
					return new ActionRescue( human );
				} else if (this.worldInfo.getPosition(targetID).getStandardURN().getURNId() != StandardEntityURN.REFUGE.getURNId()){
					List<EntityID> path = pathPlanning.getResult( agentPosition,
							targetPosition );
					if ( path != null && path.size() > 0 ) {
						return calcActionMove( path );
					}
				}
			}
			return null;
		}
		// if ( targetEntity.getStandardURN() == BLOCKADE ) {
		// 	Blockade blockade = (Blockade) targetEntity;
		// 	if ( blockade.isPositionDefined() ) {
		// 		targetEntity = this.worldInfo.getEntity( blockade.getPosition() );
		// 	}
		// }
		// if ( targetEntity instanceof Area ) {
		// 	List<EntityID> path = pathPlanning.getResult( agentPosition,
		// 			targetEntity.getID() );
		// 	if ( path != null && path.size() > 0 ) {
		// 		return calcActionMove( path );
		// 	}
		// }
		return null;
	}


	private boolean needRest( Human agent ) {
		int hp = agent.getHP();
		int damage = agent.getDamage();
		if ( hp == 0 || damage == 0 ) {
			return false;
		}
		int activeTime = ( hp / damage ) + ( ( hp % damage ) != 0 ? 1 : 0 );
		if ( this.kernelTime == -1 ) {
			try {
				this.kernelTime = this.scenarioInfo.getKernelTimesteps();
			} catch ( NoSuchConfigOptionException e ) {
				this.kernelTime = -1;
			}
		}
		return damage >= this.thresholdRest
				|| ( activeTime + this.agentInfo.getTime() ) < this.kernelTime;
	}


	private EntityID convertArea( EntityID targetID ) {
		StandardEntity entity = this.worldInfo.getEntity( targetID );
		if ( entity == null ) {
			return null;
		}
		if ( entity instanceof Human ) {
			Human human = (Human) entity;
			if ( human.isPositionDefined() ) {
				EntityID position = human.getPosition();
				if ( this.worldInfo.getEntity( position ) instanceof Area ) {
					return position;
				}
			}
		} else if ( entity instanceof Area ) {
			return targetID;
		} else if ( entity.getStandardURN() == BLOCKADE ) {
			Blockade blockade = (Blockade) entity;
			if ( blockade.isPositionDefined() ) {
				return blockade.getPosition();
			}
		}
		return null;
	}
}