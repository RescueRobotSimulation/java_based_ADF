// package TIMRAD_2025.extaction;

// import adf.core.agent.action.Action;
// import adf.core.agent.action.common.ActionMove;
// import adf.core.agent.action.common.ActionRest;
// import adf.core.agent.action.police.ActionClear;
// import adf.core.agent.communication.MessageManager;
// import adf.core.agent.develop.DevelopData;
// import adf.core.agent.info.AgentInfo;
// import adf.core.agent.info.ScenarioInfo;
// import adf.core.agent.info.WorldInfo;
// import adf.core.agent.module.ModuleManager;
// import adf.core.agent.precompute.PrecomputeData;
// import adf.core.component.extaction.ExtAction;
// import adf.core.component.module.algorithm.DynamicClustering;
// import adf.core.component.module.algorithm.PathPlanning;

// import com.google.common.collect.Lists;

// import rescuecore2.standard.entities.StandardEntityURN;
// import rescuecore2.worldmodel.EntityID;
// import rescuecore2.standard.entities.*;

// import java.util.*;
// import java.util.stream.Collectors;

// /**
//  * TIMRADExtActionClear
//  * <p>
//  * This extended action handles clearing blockades, navigating around obstacles, and handling cases where the agent gets stuck on an edge or blockade.
//  * The agent can now dynamically re-calculate paths and make smarter decisions when it encounters obstacles.
//  * </p>
//  */
// public class TIMRADExtActionClear extends ExtAction {

//     private EntityID target;
//     private PathPlanning pathPlanning;
//     private DynamicClustering invalidMove;
//     private boolean getRidof = true;
//     private boolean triggering = true;
//     private int clearDistance;
//     private int forcedMove;
//     private int thresholdRest;
//     private int kernelTime;
//     private Map<EntityID, Set<Point2D>> movePointCache;
//     private int oldClearX;
//     private int oldClearY;
//     private int count;

//     /**
//      * Constructor for TIMRADExtActionClear.
//      *
//      * @param ai             the agent info
//      * @param wi             the world info
//      * @param si             the scenario info
//      * @param moduleManager  the module manager
//      * @param developData    the development data
//      */
//     public TIMRADExtActionClear(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
//         super(ai, wi, si, moduleManager, developData);
//         this.clearDistance = si.getClearRepairDistance();
//         this.forcedMove = developData.getInteger("adf.impl.extaction.DefaultExtActionClear.forcedMove", 3);
//         this.thresholdRest = developData.getInteger("adf.impl.extaction.DefaultExtActionClear.rest", 100);

//         this.invalidMove = moduleManager.getModule("TIMRAD.PF.ExtActionClear.InvalidMove");

//         this.target = null;
//         this.movePointCache = new HashMap<>();
//         this.oldClearX = 0;
//         this.oldClearY = 0;
//         this.count = 0;

//         // Select the path planning algorithm based on the simulation mode
//         switch (si.getMode()) {
//             case PRECOMPUTATION_PHASE:
//             case PRECOMPUTED:
//             case NON_PRECOMPUTE:
//                 this.pathPlanning = moduleManager.getModule(
//                         "DefaultExtActionClear.PathPlanning",
//                         "adf.impl.module.algorithm.DijkstraPathPlanning");
//                 break;
//         }
//     }

//     /**
//      * Precompute phase of the action.
//      * Pre-calculates necessary data and updates kernel time.
//      *
//      * @param precomputeData the data from precomputation
//      * @return this extended action instance
//      */
//     @Override
//     public ExtAction precompute(PrecomputeData precomputeData) {
//         super.precompute(precomputeData);
//         if (this.getCountPrecompute() >= 2) {
//             return this;
//         }
//         this.pathPlanning.precompute(precomputeData);
//         try {
//             this.kernelTime = this.scenarioInfo.getKernelTimesteps();
//         } catch (Exception e) {
//             this.kernelTime = -1;
//         }
//         this.pathPlanning.precompute(precomputeData);
//         return this;
//     }

//     /**
//      * Resume phase of the action.
//      * Re-initializes modules from precomputed data.
//      *
//      * @param precomputeData the data from precomputation
//      * @return this extended action instance
//      */
//     @Override
//     public ExtAction resume(PrecomputeData precomputeData) {
//         super.resume(precomputeData);
//         if (this.getCountResume() >= 2) {
//             return this;
//         }
//         this.pathPlanning.resume(precomputeData);
//         try {
//             this.kernelTime = this.scenarioInfo.getKernelTimesteps();
//         } catch (Exception e) {
//             this.kernelTime = -1;
//         }
//         this.pathPlanning.resume(precomputeData);
//         this.invalidMove.resume(precomputeData);
//         return this;
//     }

//     /**
//      * Preparatory phase before main simulation steps.
//      * Sets up modules for the upcoming simulation.
//      *
//      * @return this extended action instance
//      */
//     @Override
//     public ExtAction preparate() {
//         super.preparate();
//         if (this.getCountPreparate() >= 2) {
//             return this;
//         }
//         this.pathPlanning.preparate();
//         try {
//             this.kernelTime = this.scenarioInfo.getKernelTimesteps();
//         } catch (Exception e) {
//             this.kernelTime = -1;
//         }
//         this.pathPlanning.preparate();
//         this.invalidMove.preparate();
//         return this;
//     }

//     /**
//      * Update the agent's information.
//      * This method is called before calc() to update internal states.
//      *
//      * @param messageManager the message manager used for communication updates
//      * @return this extended action instance
//      */
//     @Override
//     public ExtAction updateInfo(MessageManager messageManager) {
//         super.updateInfo(messageManager);
//         if (this.getCountUpdateInfo() >= 2) {
//             return this;
//         }
//         this.pathPlanning.updateInfo(messageManager);
//         this.invalidMove.updateInfo(messageManager);

//         // Update flags based on current state:
//         // - getRidof indicates whether the target is unreachable.
//         // - triggering indicates whether the agent is stuck.
//         this.getRidof = this.cannotreach();
//         this.triggering = this.inStuckPos();

//         return this;
//     }

//     /**
//      * Set the target for the clear action.
//      * Determines the proper target based on the entity type.
//      *
//      * @param target_id the candidate target entity ID
//      * @return this extended action instance
//      */
//     @Override
//     public ExtAction setTarget(EntityID target_id) {
//         this.target = null;
//         StandardEntity entity = this.worldInfo.getEntity(target_id);
//         if (entity != null) {
//             if (entity instanceof Road) {
//                 this.target = target_id;
//             } else if (entity.getStandardURN().equals(StandardEntityURN.BLOCKADE)) {
//                 // For blockades, set the target to the associated position
//                 this.target = ((Blockade) entity).getPosition();
//             } else if (entity instanceof Building) {
//                 this.target = target_id;
//             }
//         }
//         return this;
//     }

//     /**
//      * Main calculation method that determines the next action.
//      * The method chooses among resting, clearing, or moving based on the agent's state.
//      *
//      * @return this extended action instance with the chosen result action
//      */
//     @Override
//     public ExtAction calc() {
//         this.result = null;
//         PoliceForce policeForce = (PoliceForce) this.agentInfo.me();

//         // Check if the agent needs to rest due to damage or low HP.
//         if (this.needRest(policeForce)) {
//             List<EntityID> list = new ArrayList<>();
//             if (this.target != null) {
//                 list.add(this.target);
//             }
//             this.result = this.calcRest(policeForce, this.pathPlanning, list);
//             if (this.result != null) {
//                 return this;
//             }
//         }

//         // If no target has been set, do nothing.
//         if (this.target == null) {
//             return this;
//         }
//         EntityID agentPosition = policeForce.getPosition();

//         // If the agent is stuck, attempt to clear the blockade at the current position.
//         if (this.triggering) {
//             this.result = this.makeActionToClear(agentPosition);
//             this.cache.put(this.target, this.result);
//             return this;
//         }

//         StandardEntity targetEntity = this.worldInfo.getEntity(this.target);
//         StandardEntity positionEntity = Objects.requireNonNull(this.worldInfo.getEntity(agentPosition));
//         if (targetEntity == null || !(targetEntity instanceof Area)) {
//             return this;
//         }

//         // If the agent is on a Road, attempt to rescue nearby agents or clear blockages.
//         if (positionEntity instanceof Road) {
//             this.result = this.getRescueAction(policeForce, (Road) positionEntity);
//             if (this.result != null) {
//                 return this;
//             }
//         }

//         // If the agent is already at the target, clear the area.
//         if (agentPosition.equals(this.target)) {
//             this.result = this.getAreaClearAction(policeForce, targetEntity);
//         } else if (((Area) targetEntity).getEdgeTo(agentPosition) != null) {
//             // If the target is adjacent, move to the neighboring area.
//             this.result = this.getNeighbourPositionAction(policeForce, (Area) targetEntity);
//         } else {
//             // Otherwise, compute a path from the current position to the target.
//             List<EntityID> path = this.pathPlanning.getResult(agentPosition, this.target);
//             if (path != null && path.size() > 0) {
//                 int index = path.indexOf(agentPosition);
//                 if (index == -1) {
//                     Area area = (Area) positionEntity;
//                     for (int i = 0; i < path.size(); i++) {
//                         if (area.getEdgeTo(path.get(i)) != null) {
//                             index = i;
//                             break;
//                         }
//                     }
//                 } else if (index >= 0) {
//                     index++;
//                 }
//                 if (index >= 0 && index < (path.size())) {
//                     StandardEntity entity = this.worldInfo.getEntity(path.get(index));
//                     this.result = this.getNeighbourPositionAction(policeForce, (Area) entity);
//                     if (this.result != null && this.result.getClass() == ActionMove.class) {
//                         if (!((ActionMove) this.result).getUsePosition()) {
//                             this.result = null;
//                         }
//                     }
//                 }
//                 if (this.result == null) {
//                     // If no special action is chosen, move along the computed path.
//                     this.result = new ActionMove(path);
//                 }
//             }
//         }
//         return this;
//     }

//     /**
//      * Attempts to rescue blocked agents on a road by clearing blockades if necessary.
//      *
//      * @param police the police force agent
//      * @param road   the road entity
//      * @return an ActionClear or ActionMove, if applicable; otherwise, null
//      */
//     private Action getRescueAction(PoliceForce police, Road road) {
//         if (!road.isBlockadesDefined()) {
//             return null;
//         }
//         Collection<Blockade> blockades = this.worldInfo.getBlockades(road)
//                 .stream()
//                 .filter(Blockade::isApexesDefined)
//                 .collect(Collectors.toSet());
//         Collection<StandardEntity> agents = this.worldInfo.getEntitiesOfType(
//                 StandardEntityURN.AMBULANCE_TEAM, StandardEntityURN.FIRE_BRIGADE);

//         double policeX = police.getX();
//         double policeY = police.getY();
//         double minDistance = Double.MAX_VALUE;
//         Action moveAction = null;

//         for (StandardEntity entity : agents) {
//             Human human = (Human) entity;
//             if (!human.isPositionDefined()
//                     || human.getPosition().getValue() != road.getID().getValue()) {
//                 continue;
//             }
//             double humanX = human.getX();
//             double humanY = human.getY();
//             ActionClear actionClear = null;
//             for (Blockade blockade : blockades) {
//                 if (!this.isInside(humanX, humanY, blockade.getApexes())) {
//                     continue;
//                 }
//                 double distance = this.getDistance(policeX, policeY, humanX, humanY);
//                 if (this.intersect(policeX, policeY, humanX, humanY, road)) {
//                     Action action = this.getIntersectEdgeAction(policeX, policeY, humanX, humanY, road);
//                     if (action == null) {
//                         continue;
//                     }
//                     if (action.getClass() == ActionClear.class) {
//                         if (actionClear == null) {
//                             actionClear = (ActionClear) action;
//                             continue;
//                         }
//                         if (actionClear.getTarget() != null) {
//                             Blockade another = (Blockade) this.worldInfo.getEntity(actionClear.getTarget());
//                             if (another != null && this.intersect(blockade, another)) {
//                                 return new ActionClear(another);
//                             }
//                             int anotherDistance = this.worldInfo.getDistance(police, another);
//                             int blockadeDistance = this.worldInfo.getDistance(police, blockade);
//                             if (anotherDistance > blockadeDistance) {
//                                 return action;
//                             }
//                         }
//                         return actionClear;
//                     } else if (action.getClass() == ActionMove.class && distance < minDistance) {
//                         minDistance = distance;
//                         moveAction = action;
//                     }
//                 } else if (this.intersect(policeX, policeY, humanX, humanY, blockade)) {
//                     Vector2D vector = this.scaleClear(this.getVector(policeX, policeY, humanX, humanY));
//                     int clearX = (int) (policeX + vector.getX());
//                     int clearY = (int) (policeY + vector.getY());
//                     vector = this.scaleBackClear(vector);
//                     int startX = (int) (policeX + vector.getX());
//                     int startY = (int) (policeY + vector.getY());
//                     if (this.intersect(startX, startY, clearX, clearY, blockade)) {
//                         if (actionClear == null) {
//                             actionClear = new ActionClear(clearX, clearY, blockade);
//                         } else {
//                             if (actionClear.getTarget() != null) {
//                                 Blockade another = (Blockade) this.worldInfo.getEntity(actionClear.getTarget());
//                                 if (another != null && this.intersect(blockade, another)) {
//                                     return new ActionClear(another);
//                                 }
//                                 int distance1 = this.worldInfo.getDistance(police, another);
//                                 int distance2 = this.worldInfo.getDistance(police, blockade);
//                                 if (distance1 > distance2) {
//                                     return action;
//                                 }
//                             }
//                             return actionClear;
//                         }
//                     } else if (distance < minDistance) {
//                         minDistance = distance;
//                         moveAction = new ActionMove(Lists.newArrayList(road.getID()), (int) humanX, (int) humanY);
//                     }
//                 }
//             }
//             if (actionClear != null) {
//                 return actionClear;
//             }
//         }
//         return moveAction;
//     }

//     // Add further methods for smarter stuck handling, escape, and calculations
// }


package TIMRAD_2025.extaction;

import TIMRAD_2025.helptool.TIMRADConstants;
import TIMRAD_2025.module.complex.pf.GuidelineCreator;
import TIMRAD_2025.helptool.TIMRADRoadHelper;
import TIMRAD_2025.helptool.CircleQueue;
import TIMRAD_2025.helptool.CircleStack;
import adf.core.agent.action.Action;
import adf.core.agent.action.common.ActionMove;
import adf.core.agent.action.common.ActionRest;
import adf.core.agent.action.police.ActionClear;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.component.extaction.ExtAction;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.PoliceForce;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Line2D;
import adf.core.component.module.algorithm.PathPlanning;
import adf.core.component.module.algorithm.DynamicClustering;
import adf.core.component.module.algorithm.Clustering;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.*;
import adf.core.agent.info.ScenarioInfo;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Road;
import adf.core.agent.precompute.PrecomputeData;
import rescuecore2.standard.entities.StandardEntityURN;





import java.util.*;
import java.util.stream.Collectors;

public class TIMRADExtActionClear extends ExtAction {

  private int clearDistance, thresholdRest, kernelTime;
  private Action lastAction = null;
  protected double lastx = 0;
  protected double lasty = 0;
  private EntityID target;
  private boolean noMoveFlag = false;
  private int lastNoMoveTime = 0;
  private PathPlanning pathPlanning;
  private Clustering clustering;
  private GuidelineCreator guidelineCreator;
  private MessageManager messageManager;
  private Set<Blockade> alreadyClearedBlockades = new HashSet<>();
  private List<EntityID> clearedRoad = new ArrayList<>();
  private EntityID lastClearTarget = null;

  public TIMRADExtActionClear(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
      super(ai, wi, si, moduleManager, developData);
      this.clearDistance = si.getClearRepairDistance();
      this.thresholdRest = developData.getInteger("ActionExtClear.rest", 100);
      this.target = null;
      this.kernelTime = si.getKernelTimesteps();
      
      this.pathPlanning = moduleManager.getModule("ActionExtClear.PathPlanning", "adf.sample.module.algorithm.SamplePathPlanning");
      this.clustering = moduleManager.getModule("ActionExtClear.Clustering", "adf.sample.module.algorithm.SampleKMeans");
      this.guidelineCreator = moduleManager.getModule("GuidelineCreator.Default", TIMRADConstants.A_STAR_PATH_PLANNING);
  }

  @Override
  public ExtAction precompute(PrecomputeData precomputeData) {
      super.precompute(precomputeData);
      this.pathPlanning.precompute(precomputeData);
      this.clustering.precompute(precomputeData);
      this.guidelineCreator.precompute(precomputeData);
      return this;
  }

  @Override
  public ExtAction updateInfo(MessageManager messageManager) {
      super.updateInfo(messageManager);
      this.messageManager = messageManager;
      return this;
  }

    @Override
    public ExtAction setTarget(EntityID target_id) {
        this.target = null;
        StandardEntity entity = this.worldInfo.getEntity(target_id);
        if (entity != null) {
            if (entity instanceof Road) {
                this.target = target_id;
            } else if (entity.getStandardURN().equals(StandardEntityURN.BLOCKADE)) {
                // For blockades, set the target to the associated position
                this.target = ((Blockade) entity).getPosition();
            } else if (entity instanceof Building) {
                this.target = target_id;
            }
        }
        return this;
    }


  @Override
  public ExtAction calc() {
      // Check if the agent is stuck
      if (isStuck((Human) agentInfo.me())) {
          result = clearWhenStuck();
          if (result != null) {
              return this;
          }
      }

      // Handle random walk if the agent is not moving
      if (isNoMove()) {
          result = noMoveAction();
          if (result != null) {
              return this;
          }
      }

      // Standard action to clear blockades or move
      Action tmp = clear();
      result = tmp != null ? tmp : randomWalk();
      return this;
  }

  // Checks if the agent is stuck
  private boolean isStuck(Human agent) {
      Blockade blockade = isLocatedInBlockade(agent);
      return blockade != null && getDistance(agent.getX(), agent.getY(), blockade.getX(), blockade.getY()) < 200;
  }

  // Handles clearing blockades when the agent is stuck
  private Action clearWhenStuck() {
      Blockade blockade = isLocatedInBlockade((Human) agentInfo.me());
      if (blockade != null) {
          return new ActionClear(blockade.getID());
      } else {
          return randomWalk();
      }
  }

  // Detects if the agent is located inside a blockade
  private Blockade isLocatedInBlockade(Human agent) {
      for (EntityID entityID : worldInfo.getChanged().getChangedEntities()) {
          StandardEntity entity = worldInfo.getEntity(entityID);
          if (entity instanceof Blockade) {
              Blockade blockade = (Blockade) entity;
              if (blockade.getShape().contains(agent.getX(), agent.getY())) {
                  return blockade;
              }
          }
      }
      return null;
  }

  // Handles random movement when no specific target is found
  private Action randomWalk() {
      if (target != null) {
          StandardEntity targetEntity = worldInfo.getEntity(target);
          if (targetEntity instanceof Road) {
              pathPlanning.setDestination(target);
              List<EntityID> path = pathPlanning.calc().getResult();
              if (path != null && path.size() > 1) {
                  return new ActionMove(path);
              }
          }
      }
      return null;
  }

  // Detects if the agent hasn't moved significantly
  private boolean isNoMove() {
      double currentX = agentInfo.getX();
      double currentY = agentInfo.getY();
      int currentTime = agentInfo.getTime();
      if (Math.abs(currentX - lastx) < 200 && Math.abs(currentY - lasty) < 200) {
          if (!noMoveFlag) {
              lastNoMoveTime = currentTime;
          }
          noMoveFlag = true;
          return currentTime - lastNoMoveTime > 5;
      } else {
          noMoveFlag = false;
          lastx = currentX;
          lasty = currentY;
      }
      return false;
  }

  // Standard clearing action logic
  private Action clear() {
      PoliceForce policeForce = (PoliceForce) agentInfo.me();
      if (needRest(policeForce)) {
          return new ActionRest();
      }

      // If the agent is on the target, attempt to clear the blockade
      if (target != null) {
          Blockade blockade = findBlockadeNearby();
          if (blockade != null) {
              return new ActionClear(blockade);
          }
      }
      return null;
  }

  // Finds the closest blockade to the agent
  private Blockade findBlockadeNearby() {
      for (Blockade blockade : worldInfo.getBlockades(agentInfo.getPosition())) {
          if (blockade != null && !alreadyClearedBlockades.contains(blockade)) {
              alreadyClearedBlockades.add(blockade);
              return blockade;
          }
      }
      return null;
  }

  // Checks if the agent needs to rest based on health
  private boolean needRest(Human agent) {
      int hp = agent.getHP();
      int damage = agent.getDamage();
      return damage >= thresholdRest || (hp - damage) < 10;
  }

  // Utility to calculate the distance between two points
  private double getDistance(double fromX, double fromY, double toX, double toY) {
      return Math.hypot(toX - fromX, toY - fromY);
  }
}