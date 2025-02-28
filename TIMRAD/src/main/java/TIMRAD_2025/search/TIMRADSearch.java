package TIMRAD_2025.search;

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
import adf.core.agent.communication.standard.bundle.StandardMessagePriority;
import TIMRAD_2025.SearchAlgorithm.*;

import java.util.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;
/**
  * @author Ikegami(18th)
  * @since  2023.3.20
  * @brief  粒子フィルタとLevy Flightを動かす条件を設定し動かす
  */

public class TIMRADSearch extends Search {

    private RIOSearchLevyDistribution levyWalk = new RIOSearchLevyDistribution();
    private TIMRADSearchParticleFilter   particleFilter = new TIMRADSearchParticleFilter();

	private PathPlanning pathPlanning;
	private Clustering clustering;
    private MessageManager messageManager;
    
    private Random random = new Random(this.agentInfo.me().getID().getValue());
    private int    cycleIgnoreParticleFilter = 0;

	private EntityID result = null;
    private EntityID previousResult = null;



	public TIMRADSearch(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager,DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);

		StandardEntityURN agentURN = ai.me().getStandardURN();
		if (agentURN == AMBULANCE_TEAM) {
			this.pathPlanning = moduleManager.getModule("Search.PathPlanning.Ambulance", "RIO2023.algorithm.AstarPathPlanning");
			this.clustering = moduleManager.getModule("Search.Clustering.Ambulance", "RIO2023.algorithm.RioneKmeansPP");
		} else if (agentURN == FIRE_BRIGADE) {
			this.pathPlanning = moduleManager.getModule("Search.PathPlanning.Fire", "RIO2023.algorithm.AstarPathPlanning");
			this.clustering = moduleManager.getModule("Search.Clustering.Fire", "RIO2023.algorithm.RioneKmeansPP");
		} else if (agentURN == POLICE_FORCE) {
			this.pathPlanning = moduleManager.getModule("Search.PathPlanning.Police", "RIO2023.algorithm.AstarPathPlanning");
			this.clustering = moduleManager.getModule("Search.Clustering.Police", "RIO2023.algorithm.RioneKmeansPP");
		}
		registerModule(this.clustering);
		registerModule(this.pathPlanning);
	}

    @Override
	public Search precompute(PrecomputeData precomputeData) {
		super.precompute(precomputeData);
		if (this.getCountPrecompute() >= 2) {
			return this;
		}
		return this;
	}

	@Override
	public Search resume(PrecomputeData precomputeData) {
		super.resume(precomputeData);
		if (this.getCountResume() >= 2) {
			return this;
		}
        this.worldInfo.requestRollback();
		this.pathPlanning.resume(precomputeData);
		this.clustering.resume(precomputeData);
        this.rioSetUp(pathPlanning, clustering);
		return this;
	}

	@Override
	public Search preparate() {
		super.preparate();
		if (this.getCountPreparate() >= 2) {
			return this;
		}
		this.worldInfo.requestRollback();
		this.pathPlanning.preparate();
		this.clustering.preparate();
        this.rioSetUp(pathPlanning, clustering);
		return this;
	}
    
    private void rioSetUp(PathPlanning pp, Clustering c){
        this.levyWalk.setInfo(this.scenarioInfo, this.worldInfo, this.agentInfo).setAlgorithm(pp, c);
        this.particleFilter.setInfo(this.scenarioInfo, this.worldInfo, this.agentInfo).setAlgorithm(pp, c);
    }

	@Override
	public Search updateInfo(MessageManager messageManager) {
		super.updateInfo(messageManager);
        this.messageManager = messageManager;

        particleFilter.updateInfo(messageManager);
        levyWalk.updateInfo(messageManager);
        //叫び声が聞こえたらparticleFilterを聞こえなければ、levyWalkを使う。
        if(messageManager.getHeardAgentHelpCount() > 0 || particleFilter.WORKING) {
            if (!particleFilter.WORKING && cycleIgnoreParticleFilter <= 0) {
                particleFilter.init(); //init()でWORKING = trueになる
            }
        }
        else {
        }

		if (this.getCountUpdateInfo() >= 2) {
			return this;
		}

		return this;
	}


	private void reset(){
        //叫んでいる市民が見つかった場合とParticleFilterが10サイクル動いても見つからない場合はParticleFilterをreset
        if (!getLookScreamingCivilians(this.messageManager).isEmpty()) {
            particleFilter.reset();
        }
        else if (particleFilter.cycleOfWorking >= 10) {
            particleFilter.reset();
            cycleIgnoreParticleFilter = 5;
        }
    }

	@Override
	public Search calc() {
        //FBで助けるべき市民がいる場合、そこまで移動する。
        List<EntityID> civiliansList = new ArrayList<>(getLookScreamingCivilians(this.messageManager));
        if  (!civiliansList.isEmpty() && this.agentInfo.me() instanceof FireBrigade) {
            while(!civiliansList.isEmpty()) {
                this.result = civiliansList.get(random.nextInt(civiliansList.size()));
                if (pathPlanning.getResult(this.agentInfo.getPosition(), result) != null) {
                    particleFilter.setPreviousHeardHelp(messageManager.getHeardAgentHelpCount());
                    return this;
                }
                civiliansList.remove(result);
            }
        }

        // //ATが運ぶべき市民がいる場合、そこまで移動する。
        // civiliansList.clear();
        // if (this.agentInfo.me() instanceof AmbulanceTeam) {
        //     for (EntityID entityID : this.agentInfo.getChanged().getChangedEntities()) {
        //         if (!(this.worldInfo.getEntity(entityID) instanceof Civilian)) {
        //             continue;
        //         }
        //         if (((Human)this.worldInfo.getEntity(entityID)).getDamage() > 0) {
        //             result = entityID;
        //             if (pathPlanning.getResult(this.agentInfo.getPosition(), result) != null) {
        //                 particleFilter.setPreviousHeardHelp(messageManager.getHeardAgentHelpCount());
        //                 return this;
        //             }
        //         }
        //     }
        // }

        //まだ目的地に到達しておらず、Agentが止まっていないのであれば、同じ目的地とする。
        if (previousResult != null && !isStopping()) {
            //previousResultがArea以外ならそのresultの位置のAreaのEntityIDを取得
            if(!(this.worldInfo.getEntity(previousResult) instanceof Area)) {
                previousResult = this.worldInfo.getEntity(previousResult).getID();
            }
            //agentのいるAreaのEntityIDと対象のEntityID(対象のいるAreaのEntityID)が違うのであれば
            if (previousResult.getValue() != this.agentInfo.getPosition().getValue()) {
                this.result = this.previousResult;
                particleFilter.setPreviousHeardHelp(messageManager.getHeardAgentHelpCount());
                return this;
            }
        }

        // //ParticleFilterが動作しており、PFを無視する時間が0以下のとき
        // if (particleFilter.WORKING && cycleIgnoreParticleFilter <= 0) {
        //     //this.result = particleFilter.calc().getTarget();
        //     if (result == null) {
        //         this.result = levyWalk.calc().getTarget();
        //     }
        // }
        // //動作していないなら、LveyWalkが動いているはずなので
        // else {
        //     this.result = levyWalk.calc().getTarget();
        //     //ParticleFilterへ今回の「市民の叫び声の数」を送る
        //     particleFilter.setPreviousHeardHelp(this.messageManager.getHeardAgentHelpCount());
        // }
        this.result = levyWalk.calc().getTarget();

        // //PFで市民を見つけることができそうか
        // //見つけられないなら
        // if (!checkPossibilityFindCivilian()) {
        //     cycleIgnoreParticleFilter = 5; //音が届く距離が3000、見える距離が1000なので余裕を持って5としている
        // }
        cycleIgnoreParticleFilter--;
        reset();

        if (result != null){
            return this;
        }

        //ここより下が実行されているということは、目的地が見つかっていないと言うこと
        //瓦礫に埋まっている可能性が高い

        if (((Human)this.agentInfo.me()).isBuriednessDefined()){
            CommandFire requestRescueMe = new CommandFire(true, null, this.agentInfo.me().getID(), CommandFire.ACTION_RESCUE);//1はACTION_MOVE
            //MessageFireBrigade messageFireBrigade = new MessageFireBrigade(true, null, MessageFireBrigade.ACTION_REFILL, this.agentInfo.me().getID());
            this.messageManager.addMessage(requestRescueMe, true);
        }
        CommandPolice requestExtinguishBlockade = new CommandPolice(true, null , this.agentInfo.getPosition(), CommandPolice.ACTION_CLEAR);//1はACTION_MOVE
        //MessagePoliceForce messagePoliceForce = new MessagePoliceForce(true, null, MessagePoliceForce.ACTION_CLEAR, this.agentInfo.me().getID());
        this.messageManager.addMessage(requestExtinguishBlockade, true);
      
		return this;
	}

	@Override
	public EntityID getTarget() {
        this.previousResult = result;
		return this.result;
	}

    // private EntityID allSearch() {
    //     EntityID targetID = null;
    //     return targetID;
    // }

    /**叫んだと思われる市民を調べる */
    private Set<EntityID> getLookScreamingCivilians(MessageManager messageManager) {
        Set<EntityID>  lookScreamingCivilians = new HashSet<>();
        List<EntityID> fireBrigadeIDs = new ArrayList<>(this.worldInfo.getEntityIDsOfType(StandardEntityURN.FIRE_BRIGADE));


        for (EntityID entityID : this.agentInfo.getChanged().getChangedEntities()){
            //entityIDが市民でなければ、continue
            if (!(this.worldInfo.getEntity(entityID) instanceof Civilian)){
                continue;
            }
            //市民の埋没度が0(埋まっていない)であれば、その市民が叫んでいないと判断しcontinue
            if (((Human)this.worldInfo.getEntity(entityID)).getBuriedness() == 0) {
                continue;
            }
            //continueされていないので、叫んでいる市民を確認できたと判断
            lookScreamingCivilians.add(entityID);

            //FBに市民の情報を送信
            if (fireBrigadeIDs.isEmpty()){
                CommandFire requestRescueCivilian = new CommandFire(true, null, entityID, 4);//4はACTION_RESCUE
                //MessageCivilian messageCivilian = new MessageCivilian(true, StandardMessagePriority.NORMAL, (Civilian)this.worldInfo.getEntity(entityID));
                messageManager.addMessage(requestRescueCivilian, true);
            }
            else {
                CommandFire requestRescueCivilian = new CommandFire(false, fireBrigadeIDs.get(this.random.nextInt(fireBrigadeIDs.size())), entityID, 4);//4はACTION_RESCUE
                //MessageCivilian messageCivilian = new MessageCivilian(true, StandardMessagePriority.NORMAL, (Civilian)this.worldInfo.getEntity(entityID));
                messageManager.addMessage(requestRescueCivilian, true);
            }
        }

        return lookScreamingCivilians;
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

    private boolean checkPossibilityFindCivilian() {
        //PFが10サイクル動いても見つからない場合(市民を見つけたらcycleOfWorking = 0 に)
        if(particleFilter.cycleOfWorking > 10) {
            return false;
        }

        return true;
    }
}
