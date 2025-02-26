package TIMRAD.module.complex.search;

import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.WorldInfo;
import TIMRAD.TIMRADConstants;
import TIMRAD.util.Util;
import TIMRAD.world.TIMRADWorldHelper;
import TIMRAD.world.entity.TIMRADBuilding;
import TIMRAD.world.entity.MrlRoad;
import TIMRAD.world.entity.Path;
import TIMRAD.world.helper.CivilianHelper;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.util.*;

/**
 * @author Shiva
 */

public class TIMRADSearchDecisionMaker {
    private Set<EntityID> shouldDiscoverBuildings;
    private Set<EntityID> shouldFindCivilians;
    private Set<EntityID> unreachableCivilians;
    private MrlWorldHelper world;
    private AgentInfo agentInfo;
    private WorldInfo worldInfo;
    private Map<EntityID, Integer> notVisitable;
    private Set<EntityID> validBuildings;
    private Map<EntityID, Double> searchPriorityCache;

    public TIMRADSearchDecisionMaker(WorldHelper world, AgentInfo ai, WorldInfo wi) // WorldHelper --> TIMRADWorldHelper
 {
        this.world = world;
        agentInfo = ai;
        worldInfo = wi;
        notVisitable = new HashMap<>();
        searchPriorityCache = new HashMap<>();
    }

    public void initialize() {
        shouldFindCivilians = new HashSet<>();
        shouldDiscoverBuildings = new HashSet<>();
        unreachableCivilians = new HashSet<>();
        validBuildings = new HashSet<>();
    }

    public void update() {
        validBuildings.addAll(world.getBuildingIDs());
        setShouldFindCivilians();
        shouldFindCivilians.removeAll(unreachableCivilians);
        setShouldDiscoverBuildings();
        removeZeroBrokennessBuildings();
        removeBurningBuildings();
        removeVisitedBuildings();
        if (!(agentInfo.me() instanceof PoliceForce)) {
            removeUnreachableBuildings();
        }
        updateCivilianPossibleValues();
    }

    private void removeUnreachableBuildings() {
        shouldDiscoverBuildings.removeIf(bID -> !world.getBuilding(bID).isVisitable()); // getBuilding --> getTIMRADBuilding
    }

    private void removeVisitedBuildings() {
        shouldDiscoverBuildings.removeAll(world.getVisitedBuildings());
    }

    private void removeBurningBuildings() {
        shouldDiscoverBuildings.removeIf(buildingID -> {
            Building building = (Building) worldInfo.getEntity(buildingID);
            return building.isFierynessDefined() && building.getFieryness() > 1;
        });
    }

    private void setShouldFindCivilians() {
        shouldFindCivilians.clear();
        for (EntityID civId : world.getAllCivilians()) {
            Civilian civilian = (Civilian) world.getEntity(civId);
            if (civilian == null || !civilian.isPositionDefined()) {
                shouldFindCivilians.add(civId);
            }
        }
    }

    private void setShouldDiscoverBuildings() {
        CivilianHelper civilianHelper = world.getHelper(CivilianHelper.class);
        shouldDiscoverBuildings.clear();
        for (EntityID civId : shouldFindCivilians) {
            for (EntityID possibleBuildingID : civilianHelper.getPossibleBuildings(civId)) {
                Building timradBuilding = world.getBuilding(possibleBuildingID); // Building --> TimRADBuilding و getBuilding --> getTIMRADBuilding

                timradBuilding.addCivilianPossibly(civId);
                shouldDiscoverBuildings.add(possibleBuildingID);
            }
        }
    }

    private void removeZeroBrokennessBuildings() {
        shouldDiscoverBuildings.removeIf(buildingID -> {
            Building building = (Building) worldInfo.getEntity(buildingID);
            return building.isBrokennessDefined() && building.getBrokenness() == 0;
        });
    }

    private void updateCivilianPossibleValues() {
        for (EntityID bID : shouldDiscoverBuildings) {
            Building timradBuilding = world.getBuilding(bID); // Building --> TimRADBuilding و getBuilding --> getTIMRADBuilding
            double civilianPossibleValue = timradBuilding.getCivilianPossibly().size();
            if (civilianPossibleValue > 0) {
                double distance = Util.distance(worldInfo.getLocation(agentInfo.getID()), worldInfo.getLocation(timradBuilding.getSelfBuilding()));
                double timeToArrive = distance / TIMRADConstants.MEAN_VELOCITY_OF_MOVING; // Constants --> TIMRADConstants
                civilianPossibleValue /= Math.max(timeToArrive, 1);
                searchPriorityCache.put(bID, civilianPossibleValue);
            }
        }
    }

    public Area getNextArea() {
        return shouldDiscoverBuildings.stream()
                .max(Comparator.comparingDouble(searchPriorityCache::get))
                .map(id -> world.getEntity(id, Area.class))
                .orElse(null);
    }
}
