public class TIMRADSearchStrategy {
    private Set<Building> buildings;
    private Building targetBuilding;
    private PathPlanning pathPlanning;
    private Set<Building> discoveredBuildings;
    private Map<EntityID, Integer> cachedPaths;
    private List<EntityID> moveLogs;
    private int maxRetries = 3;

    public CivilianSearchStrategy(Set<Building> buildings, PathPlanning pathPlanning) {
        this.buildings = new HashSet<>(buildings);
        this.pathPlanning = pathPlanning;
        this.discoveredBuildings = new HashSet<>();
        this.cachedPaths = new HashMap<>();
        this.moveLogs = new ArrayList<>();
    }

    public void searchPath() {
        prioritizeBuildings();
        if (targetBuilding == null || isBuildingInvalid(targetBuilding)) {
            selectNextTarget();
        }
        moveToTarget();
    }

    private void prioritizeBuildings() {
        buildings.removeIf(b -> b.isBurning() || discoveredBuildings.contains(b));
        buildings = buildings.stream()
                .sorted(Comparator.comparingInt(this::evaluateBuildingPriority))
                .collect(Collectors.toCollection(LinkedHashSet::new));
    }

    private int evaluateBuildingPriority(Building b) {
        int distance = cachedPaths.getOrDefault(b.getID(), simpleTTA(b));
        int priority = (b.hasCivilians() ? -50 : 0) + (b.isBurning() ? 100 : 0) + distance;
        cachedPaths.put(b.getID(), distance);
        return priority;
    }

    private void selectNextTarget() {
        targetBuilding = buildings.stream().findFirst().orElse(null);
    }

    private void moveToTarget() {
        if (targetBuilding == null) return;
        List<EntityID> path = pathPlanning.getResult(targetBuilding.getID());
        if (path == null || path.isEmpty()) {
            buildings.remove(targetBuilding);
            selectNextTarget();
            return;
        }
        executeMovement(path);
    }

    private void executeMovement(List<EntityID> path) {
        moveLogs.add(path.get(path.size() - 1));
        if (moveLogs.size() > maxRetries && moveLogs.subList(moveLogs.size() - maxRetries, moveLogs.size()).stream().distinct().count() == 1) {
            buildings.remove(targetBuilding);
            selectNextTarget();
        }
    }

    private boolean isBuildingInvalid(Building b) {
        return discoveredBuildings.contains(b) || b.isBurning();
    }
}
