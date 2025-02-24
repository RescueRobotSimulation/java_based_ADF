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
import adf.core.component.module.algorithm.DynamicClustering;
import adf.core.component.module.algorithm.PathPlanning;

import com.google.common.collect.Lists;

import static rescuecore2.standard.entities.StandardEntityURN.REFUGE;

import java.util.*;
// import java.util.ArrayList;
// import java.util.Collection;
// import java.util.HashMap;
// import java.util.HashSet;
// import java.util.List;
// import java.util.Map;
// import java.util.Objects;
// import java.util.Set;
import java.util.stream.Collectors;

import rescuecore2.config.NoSuchConfigOptionException;

import rescuecore2.misc.geometry.*;
// import rescuecore2.misc.geometry.GeometryTools2D;
// import rescuecore2.misc.geometry.Line2D;
// import rescuecore2.misc.geometry.Point2D;
// import rescuecore2.misc.geometry.Vector2D;
import rescuecore2.standard.entities.*;
// import rescuecore2.standard.entities.Area;
// import rescuecore2.standard.entities.Blockade;
// import rescuecore2.standard.entities.Building;
// import rescuecore2.standard.entities.Edge;
// import rescuecore2.standard.entities.Human;
// import rescuecore2.standard.entities.PoliceForce;
// import rescuecore2.standard.entities.Road;
// import rescuecore2.standard.entities.StandardEntity;
// import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;

/**
 * TIMRADExtActionClear
 * 
 * This class implements the extended action for clearing blockades.
 * It determines the target to be cleared and selects the appropriate action 
 * (move, clear, or rest) based on the agent's situation.
 * <p>
 * Note: All original comments have been preserved and additional comments 
 * have been added for better readability.
 * </p>
 */
public class TIMRADExtActionClear extends ExtAction {

  /* Entity ID of the entity that the PF action is targeted for. */
  private EntityID target;

  /* Path planning module which determines the movement path. */
  private PathPlanning pathPlanning;

  /* 
   * Dynamic Clustering module which specifies a new path when the movement fails.
   */
  private DynamicClustering invalidMove;

  /* 
   * Flag that, if false, opens another piece of rubble.
   */
  private boolean getRidof = true;

  /* 
   * Flag triggered when the agent is physically stuck in a blockade,
   * as indicated by the inStuckPos() method.
   */
  private boolean triggering = true;

  // Clear distance (how far the agent can clear)
  private int clearDistance;
  // Maximum allowed forced moves before taking alternative action
  private int forcedMove;
  // Threshold of damage at which the agent needs to rest
  private int thresholdRest;
  // Total timesteps defined by the kernel (simulation)
  private int kernelTime;
  // Cache for move points per road
  private Map<EntityID, Set<Point2D>> movePointCache;
  // Variables for caching last clear point coordinates
  private int oldClearX;
  private int oldClearY;
  // Counter for forced moves
  private int count;

  /**
   * Constructor for TIMRADExtActionClear.
   *
   * @param ai             the agent info
   * @param wi             the world info
   * @param si             the scenario info
   * @param moduleManager  the module manager
   * @param developData    the development data
   */
  public TIMRADExtActionClear(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
    super(ai, wi, si, moduleManager, developData);
    this.clearDistance = si.getClearRepairDistance();
    this.forcedMove = developData.getInteger("adf.impl.extaction.DefaultExtActionClear.forcedMove", 3);
    this.thresholdRest = developData.getInteger("adf.impl.extaction.DefaultExtActionClear.rest", 100);

    // Initialize the invalid move clustering module
    this.invalidMove = moduleManager.getModule("TIMRAD.PF.ExtActionClear.InvalidMove");

    this.target = null;
    this.movePointCache = new HashMap<>();
    this.oldClearX = 0;
    this.oldClearY = 0;
    this.count = 0;

    // Select the path planning algorithm based on the simulation mode
    switch (si.getMode()) {
      case PRECOMPUTATION_PHASE:
      case PRECOMPUTED:
      case NON_PRECOMPUTE:
        this.pathPlanning = moduleManager.getModule(
            "DefaultExtActionClear.PathPlanning",
            "adf.impl.module.algorithm.DijkstraPathPlanning");
        break;
    }
  }

  /**
   * Precompute phase of the action.
   * Pre-calculates necessary data and updates kernel time.
   *
   * @param precomputeData the data from precomputation
   * @return this extended action instance
   */
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
    // Calling precompute again as required by module design
    this.pathPlanning.precompute(precomputeData);
    return this;
  }

  /**
   * Resume phase of the action.
   * Re-initializes modules from precomputed data.
   *
   * @param precomputeData the data from precomputation
   * @return this extended action instance
   */
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
    // Resume the modules again as per design
    this.pathPlanning.resume(precomputeData);
    this.invalidMove.resume(precomputeData);

    return this;
  }

  /**
   * Preparatory phase before main simulation steps.
   * Sets up modules for the upcoming simulation.
   *
   * @return this extended action instance
   */
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
    // Call preparate on modules as needed
    this.pathPlanning.preparate();
    this.invalidMove.preparate();
    return this;
  }

  /**
   * Update the agent's information.
   * This method is called before calc() to update internal states.
   *
   * @param messageManager the message manager used for communication updates
   * @return this extended action instance
   */
  @Override
  public ExtAction updateInfo(MessageManager messageManager) {
    super.updateInfo(messageManager);
    if (this.getCountUpdateInfo() >= 2) {
      return this;
    }
    this.pathPlanning.updateInfo(messageManager);
    this.invalidMove.updateInfo(messageManager);

    // Update flags based on current state:
    // - getRidof indicates whether the target is unreachable.
    // - triggering indicates whether the agent is stuck.
    this.getRidof = this.cannotreach();
    this.triggering = this.inStuckPos();

    return this;
  }

  /**
   * Set the target for the clear action.
   * Determines the proper target based on the entity type.
   *
   * @param target_id the candidate target entity ID
   * @return this extended action instance
   */
  @Override
  public ExtAction setTarget(EntityID target_id) {
    this.target = null;
    StandardEntity entity = this.worldInfo.getEntity(target_id);
    if (entity != null) {
      if (entity instanceof Road) {
        System.out.println("road");
        this.target = target_id;
      } else if (entity.getStandardURN().equals(StandardEntityURN.BLOCKADE)) {
        System.out.println("blochade");
        // For blockades, set the target to the associated position
        this.target = ((Blockade) entity).getPosition();
      } else if (entity instanceof Building) {
        System.out.println("building");
        this.target = target_id;
      }
    }
    System.out.println(this.target);
    return this;
  }

  /**
   * Main calculation method that determines the next action.
   * The method chooses among resting, clearing, or moving based on the agent's state.
   *
   * @return this extended action instance with the chosen result action
   */
  @Override
  public ExtAction calc() {
    this.result = null;
    PoliceForce policeForce = (PoliceForce) this.agentInfo.me();
    System.out.println(policeForce);

    // Check if the agent needs to rest due to damage or low HP.
    if (this.needRest(policeForce)) {
      List<EntityID> list = new ArrayList<>();
      if (this.target != null) {
        list.add(this.target);
      }
      this.result = this.calcRest(policeForce, this.pathPlanning, list);
      if (this.result != null) {
        return this;
      }
    }

    // If no target has been set, do nothing.
    if (this.target == null) {
      return this;
    }
    EntityID agentPosition = policeForce.getPosition();
    // If the agent is stuck, attempt to clear the blockade at the current position.
    if (this.triggering) {
      this.result = this.makeActionToClear(agentPosition);
      this.cache.put(this.target, this.result);
      return this;
    }
    StandardEntity targetEntity = this.worldInfo.getEntity(this.target);
    StandardEntity positionEntity = Objects.requireNonNull(this.worldInfo.getEntity(agentPosition));
    if (targetEntity == null || !(targetEntity instanceof Area)) {
      return this;
    }
    // If the agent is on a Road, attempt to rescue nearby agents or clear blockages.
    if (positionEntity instanceof Road) {
      this.result = this.getRescueAction(policeForce, (Road) positionEntity);
      if (this.result != null) {
        return this;
      }
    }
    // If the agent is already at the target, clear the area.
    if (agentPosition.equals(this.target)) {
      this.result = this.getAreaClearAction(policeForce, targetEntity);
    } else if (((Area) targetEntity).getEdgeTo(agentPosition) != null) {
      // If the target is adjacent, move to the neighboring area.
      this.result = this.getNeighbourPositionAction(policeForce, (Area) targetEntity);
    } else {
      // Otherwise, compute a path from the current position to the target.
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
        } else if (index >= 0) {
          index++;
        }
        if (index >= 0 && index < (path.size())) {
          StandardEntity entity = this.worldInfo.getEntity(path.get(index));
          this.result = this.getNeighbourPositionAction(policeForce, (Area) entity);
          if (this.result != null && this.result.getClass() == ActionMove.class) {
            if (!((ActionMove) this.result).getUsePosition()) {
              this.result = null;
            }
          }
        }
        if (this.result == null) {
          // If no special action is chosen, move along the computed path.
          this.result = new ActionMove(path);
        }
      }
    }
    return this;
  }

  /**
   * Attempts to rescue blocked agents on a road by clearing blockades if necessary.
   *
   * @param police the police force agent
   * @param road   the road entity
   * @return an ActionClear or ActionMove, if applicable; otherwise, null
   */
  private Action getRescueAction(PoliceForce police, Road road) {
    if (!road.isBlockadesDefined()) {
      return null;
    }
    Collection<Blockade> blockades = this.worldInfo.getBlockades(road)
        .stream()
        .filter(Blockade::isApexesDefined)
        .collect(Collectors.toSet());
    Collection<StandardEntity> agents = this.worldInfo.getEntitiesOfType(
        StandardEntityURN.AMBULANCE_TEAM, StandardEntityURN.FIRE_BRIGADE);

    double policeX = police.getX();
    double policeY = police.getY();
    double minDistance = Double.MAX_VALUE;
    Action moveAction = null;
    for (StandardEntity entity : agents) {
      Human human = (Human) entity;
      if (!human.isPositionDefined() || human.getPosition().getValue() != road.getID().getValue()) {
        continue;
      }
      double humanX = human.getX();
      double humanY = human.getY();
      ActionClear actionClear = null;
      for (Blockade blockade : blockades) {
        if (!this.isInside(humanX, humanY, blockade.getApexes())) {
          continue;
        }
        double distance = this.getDistance(policeX, policeY, humanX, humanY);
        if (this.intersect(policeX, policeY, humanX, humanY, road)) {
          Action action = this.getIntersectEdgeAction(policeX, policeY, humanX, humanY, road);
          if (action == null) {
            continue;
          }
          if (action.getClass() == ActionClear.class) {
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
          } else if (action.getClass() == ActionMove.class && distance < minDistance) {
            minDistance = distance;
            moveAction = action;
          }
        } else if (this.intersect(policeX, policeY, humanX, humanY, blockade)) {
          Vector2D vector = this.scaleClear(this.getVector(policeX, policeY, humanX, humanY));
          int clearX = (int) (policeX + vector.getX());
          int clearY = (int) (policeY + vector.getY());
          vector = this.scaleBackClear(vector);
          int startX = (int) (policeX + vector.getX());
          int startY = (int) (policeY + vector.getY());
          if (this.intersect(startX, startY, clearX, clearY, blockade)) {
            if (actionClear == null) {
              actionClear = new ActionClear(clearX, clearY, blockade);
            } else {
              if (actionClear.getTarget() != null) {
                Blockade another = (Blockade) this.worldInfo.getEntity(actionClear.getTarget());
                if (another != null && this.intersect(blockade, another)) {
                  return new ActionClear(another);
                }
                int distance1 = this.worldInfo.getDistance(police, another);
                int distance2 = this.worldInfo.getDistance(police, blockade);
                if (distance1 > distance2) {
                  return action;
                }
              }
              return actionClear;
            }
          } else if (distance < minDistance) {
            minDistance = distance;
            moveAction = new ActionMove(Lists.newArrayList(road.getID()), (int) humanX, (int) humanY);
          }
        }
      }
      if (actionClear != null) {
        return actionClear;
      }
    }
    return moveAction;
  }

  /**
   * Generates an action to clear the target area if it is a building or road with blockades.
   *
   * @param police        the police force agent
   * @param targetEntity  the target entity (expected to be an Area)
   * @return an ActionClear if within range; otherwise, an ActionMove toward the blockade or null
   */
  private Action getAreaClearAction(PoliceForce police, StandardEntity targetEntity) {
    if (targetEntity instanceof Building) {
      return null;
    }
    Road road = (Road) targetEntity;
    if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) {
      return null;
    }
    Collection<Blockade> blockades = this.worldInfo.getBlockades(road)
        .stream()
        .filter(Blockade::isApexesDefined)
        .collect(Collectors.toSet());
    int minDistance = Integer.MAX_VALUE;
    Blockade clearBlockade = null;
    // Compare distances between blockades to choose the one to clear
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
        return new ActionMove(Lists.newArrayList(police.getPosition()), clearBlockade.getX(), clearBlockade.getY());
      }
    }
    // If no single blockade is clearly chosen, determine the closest blockade point
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
      if (minPointDistance < this.clearDistance) {
        Vector2D vector = this.scaleClear(this.getVector(agentX, agentY, clearX, clearY));
        clearX = (int) (agentX + vector.getX());
        clearY = (int) (agentY + vector.getY());
        return new ActionClear(clearX, clearY, clearBlockade);
      }
      return new ActionMove(Lists.newArrayList(police.getPosition()), clearX, clearY);
    }
    return null;
  }

  /**
   * Computes an action to move to a neighboring area if the target is adjacent.
   *
   * @param police the police force agent
   * @param target the target area
   * @return an ActionMove or ActionClear depending on the situation; otherwise, a default move action
   */
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
      if (road.isBlockadesDefined() && road.getBlockades().size() > 0) {
        double midX = (edge.getStartX() + edge.getEndX()) / 2;
        double midY = (edge.getStartY() + edge.getEndY()) / 2;
        if (this.intersect(agentX, agentY, midX, midY, road)) {
          return this.getIntersectEdgeAction(agentX, agentY, edge, road);
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
      if (clearBlockade != null && minPointDistance < this.clearDistance) {
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
        return new ActionClear(clearX, clearY, clearBlockade);
      }
    }
    // Default move action if none of the above conditions are met
    return new ActionMove(Lists.newArrayList(position.getID(), target.getID()));
  }

  /**
   * Overloaded helper: Get intersect edge action given an Edge.
   *
   * @param agentX the agent's x-coordinate
   * @param agentY the agent's y-coordinate
   * @param edge   the edge of the current area
   * @param road   the road entity containing the edge
   * @return an action based on the intersection with the edge
   */
  private Action getIntersectEdgeAction(double agentX, double agentY, Edge edge, Road road) {
    double midX = (edge.getStartX() + edge.getEndX()) / 2;
    double midY = (edge.getStartY() + edge.getEndY()) / 2;
    return this.getIntersectEdgeAction(agentX, agentY, midX, midY, road);
  }

  /**
   * Computes an action based on the best move point along an edge.
   *
   * @param agentX the agent's x-coordinate
   * @param agentY the agent's y-coordinate
   * @param pointX the x-coordinate of the candidate point
   * @param pointY the y-coordinate of the candidate point
   * @param road   the road entity
   * @return an ActionClear or ActionMove based on intersections; otherwise, a fallback action
   */
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

  /**
   * Checks if two points are equal within a default range.
   *
   * @param p1X the first point's x-coordinate
   * @param p1Y the first point's y-coordinate
   * @param p2X the second point's x-coordinate
   * @param p2Y the second point's y-coordinate
   * @return true if the points are approximately equal; otherwise, false
   */
  private boolean equalsPoint(double p1X, double p1Y, double p2X, double p2Y) {
    return this.equalsPoint(p1X, p1Y, p2X, p2Y, 1000.0D);
  }

  /**
   * Checks if two points are equal within a given range.
   *
   * @param p1X   the first point's x-coordinate
   * @param p1Y   the first point's y-coordinate
   * @param p2X   the second point's x-coordinate
   * @param p2Y   the second point's y-coordinate
   * @param range the acceptable range difference
   * @return true if the points are approximately equal; otherwise, false
   */
  private boolean equalsPoint(double p1X, double p1Y, double p2X, double p2Y, double range) {
    return (p2X - range < p1X && p1X < p2X + range)
        && (p2Y - range < p1Y && p1Y < p2Y + range);
  }

  /**
   * Determines if a point is inside the polygon defined by the given apex array.
   *
   * @param pX    the x-coordinate of the point
   * @param pY    the y-coordinate of the point
   * @param apex  the apex array of the polygon
   * @return true if the point is inside; otherwise, false
   */
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

  /**
   * Checks whether a line between two points intersects any edge of an area.
   *
   * @param agentX the starting x-coordinate
   * @param agentY the starting y-coordinate
   * @param pointX the ending x-coordinate
   * @param pointY the ending y-coordinate
   * @param area   the area to check against
   * @return true if an intersection is found; otherwise, false
   */
  private boolean intersect(double agentX, double agentY, double pointX, double pointY, Area area) {
    for (Edge edge : area.getEdges()) {
      double startX = edge.getStartX();
      double startY = edge.getStartY();
      double endX = edge.getEndX();
      double endY = edge.getEndY();
      if (java.awt.geom.Line2D.linesIntersect(agentX, agentY, pointX, pointY,
          startX, startY, endX, endY)) {
        double midX = (edge.getStartX() + edge.getEndX()) / 2;
        double midY = (edge.getStartY() + edge.getEndY()) / 2;
        if (!equalsPoint(pointX, pointY, midX, midY)
            && !equalsPoint(agentX, agentY, midX, midY)) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Determines whether two blockades intersect.
   *
   * @param blockade the first blockade
   * @param another  the second blockade
   * @return true if they intersect; otherwise, false
   */
  private boolean intersect(Blockade blockade, Blockade another) {
    if (blockade.isApexesDefined() && another.isApexesDefined()) {
      int[] apexes0 = blockade.getApexes();
      int[] apexes1 = another.getApexes();
      for (int i = 0; i < (apexes0.length - 2); i += 2) {
        for (int j = 0; j < (apexes1.length - 2); j += 2) {
          if (java.awt.geom.Line2D.linesIntersect(apexes0[i], apexes0[i + 1],
              apexes0[i + 2], apexes0[i + 3],
              apexes1[j], apexes1[j + 1],
              apexes1[j + 2], apexes1[j + 3])) {
            return true;
          }
        }
      }
      for (int i = 0; i < (apexes0.length - 2); i += 2) {
        if (java.awt.geom.Line2D.linesIntersect(apexes0[i], apexes0[i + 1],
            apexes0[i + 2], apexes0[i + 3],
            apexes1[apexes1.length - 2], apexes1[apexes1.length - 1],
            apexes1[0], apexes1[1])) {
          return true;
        }
      }
      for (int j = 0; j < (apexes1.length - 2); j += 2) {
        if (java.awt.geom.Line2D.linesIntersect(apexes0[apexes0.length - 2],
            apexes0[apexes0.length - 1],
            apexes0[0], apexes0[1],
            apexes1[j], apexes1[j + 1],
            apexes1[j + 2], apexes1[j + 3])) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Checks if a line between two points intersects a blockade.
   *
   * @param agentX   the starting x-coordinate
   * @param agentY   the starting y-coordinate
   * @param pointX   the ending x-coordinate
   * @param pointY   the ending y-coordinate
   * @param blockade the blockade entity
   * @return true if there is an intersection; otherwise, false
   */
  private boolean intersect(double agentX, double agentY, double pointX, double pointY, Blockade blockade) {
    List<Line2D> lines = GeometryTools2D.pointsToLines(
        GeometryTools2D.vertexArrayToPoints(blockade.getApexes()), true);
    for (Line2D line : lines) {
      Point2D start = line.getOrigin();
      Point2D end = line.getEndPoint();
      double startX = start.getX();
      double startY = start.getY();
      double endX = end.getX();
      double endY = end.getY();
      if (java.awt.geom.Line2D.linesIntersect(agentX, agentY, pointX, pointY,
          startX, startY, endX, endY)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Computes the Euclidean distance between two points.
   *
   * @param fromX the starting x-coordinate
   * @param fromY the starting y-coordinate
   * @param toX   the ending x-coordinate
   * @param toY   the ending y-coordinate
   * @return the distance between the points
   */
  private double getDistance(double fromX, double fromY, double toX, double toY) {
    double dx = toX - fromX;
    double dy = toY - fromY;
    return Math.hypot(dx, dy);
  }

  /**
   * Computes the angle between two vectors.
   *
   * @param v1 the first vector
   * @param v2 the second vector
   * @return the angle in radians (positive if v2 is to the left of v1)
   */
  private double getAngle(Vector2D v1, Vector2D v2) {
    double flag = (v1.getX() * v2.getY()) - (v1.getY() * v2.getX());
    double angle = Math.acos(((v1.getX() * v2.getX()) + (v1.getY() * v2.getY()))
        / (v1.getLength() * v2.getLength()));
    if (flag > 0) {
      return angle;
    }
    if (flag < 0) {
      return -1 * angle;
    }
    return 0.0D;
  }

  /**
   * Returns a vector from one point to another.
   *
   * @param fromX the starting x-coordinate
   * @param fromY the starting y-coordinate
   * @param toX   the ending x-coordinate
   * @param toY   the ending y-coordinate
   * @return the resulting vector
   */
  private Vector2D getVector(double fromX, double fromY, double toX, double toY) {
    return (new Point2D(toX, toY)).minus(new Point2D(fromX, fromY));
  }

  /**
   * Scales a vector to the clear distance.
   *
   * @param vector the original vector
   * @return the scaled vector
   */
  private Vector2D scaleClear(Vector2D vector) {
    return vector.normalised().scale(this.clearDistance);
  }

  /**
   * Scales a vector in the opposite direction for clearing adjustments.
   *
   * @param vector the original vector
   * @return the reversed and scaled vector
   */
  private Vector2D scaleBackClear(Vector2D vector) {
    return vector.normalised().scale(-510);
  }

  /**
   * Computes and returns a set of candidate move points on the given road.
   *
   * @param road the road entity
   * @return a set of Point2D representing candidate move points
   */
  private Set<Point2D> getMovePoints(Road road) {
    Set<Point2D> points = this.movePointCache.get(road.getID());
    if (points == null) {
      points = new HashSet<>();
      int[] apex = road.getApexList();
      // Calculate midpoints between apex pairs and add if inside the polygon
      for (int i = 0; i < apex.length; i += 2) {
        for (int j = i + 2; j < apex.length; j += 2) {
          double midX = (apex[i] + apex[j]) / 2;
          double midY = (apex[i + 1] + apex[j + 1]) / 2;
          if (this.isInside(midX, midY, apex)) {
            points.add(new Point2D(midX, midY));
          }
        }
      }
      // Remove points that coincide with edge midpoints
      for (Edge edge : road.getEdges()) {
        double midX = (edge.getStartX() + edge.getEndX()) / 2;
        double midY = (edge.getStartY() + edge.getEndY()) / 2;
        points.remove(new Point2D(midX, midY));
      }
      this.movePointCache.put(road.getID(), points);
    }
    return points;
  }

  /**
   * Determines whether the agent needs to rest based on its HP and damage.
   *
   * @param agent the agent (Human) entity
   * @return true if the agent should rest; otherwise, false
   */
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

  /**
   * Calculates a rest action for the agent.
   * The agent seeks a refuge if it needs to rest.
   *
   * @param human         the human agent
   * @param pathPlanning  the path planning module
   * @param targets       a collection of target EntityIDs (if any)
   * @return an ActionRest or ActionMove leading to refuge; otherwise, null
   */
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
        // remove failed refuge if size does not change
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
   * Determines if the agent cannot reach the target.
   * Uses the invalidMove clustering module to decide.
   *
   * @return true if the agent is in an invalid move cluster; otherwise, false
   */
  private boolean cannotreach() {
    final EntityID currentposition = this.agentInfo.getID();
    this.invalidMove.calc();
    return this.invalidMove.getClusterIndex(currentposition) >= 0;
  }

  /* Escape to nearest refuge
   * @return results the refuge if it's not null.
   */
  // private EntityID bestRefugeSeeker() {
  //   final EntityID currentposition = this.agentInfo.getID();
  //   final Optional<EntityID> ret = this.worldInfo.getEntityIDsOfType(REFUGE)
  //       .stream()
  //       .min((initR, finR) -> {
  //         final double x1 = this.worldInfo.getDistance(currentposition, initR) +
  //           this.worldInfo.getDistance(initR, this.target);
  //         final double x2 = this.worldInfo.getDistance(finR, this.target) + 
  //           this.worldInfo.getDistance(finR, this.target);
  //         return Double.compare(x1, x2);
  //       });
  //   return ret.orElse(null);
  // }
}
