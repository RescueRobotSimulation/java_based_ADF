package TIMRAD_2025.helptool;

import TIMRAD_2025.helptool.TIMRADConstants;
import TIMRAD_2025.debugger.DebugHelper;
import TIMRAD_2025.helptool.PolygonScaler;
import TIMRAD_2025.module.algorithm.CompositeConvexHull;
import TIMRAD_2025.helptool.Ruler;
import TIMRAD_2025.standard.simplePartition.Line;
import TIMRAD_2025.helptool.GraphHelper;
import TIMRAD_2025.helptool.MyEdge;
import TIMRAD_2025.helptool.object.*;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.MessageUtil;
import adf.core.agent.communication.standard.bundle.StandardMessage;
import adf.core.agent.communication.standard.bundle.StandardMessagePriority;
import adf.core.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.core.agent.communication.standard.bundle.information.MessageBuilding;
import adf.core.agent.communication.standard.bundle.information.MessageCivilian;
import adf.core.agent.communication.standard.bundle.information.MessageRoad;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.communication.CommunicationMessage;
import adf.core.component.communication.util.BitStreamReader;
import adf.core.component.module.AbstractModule;
import adf.core.launcher.ConsoleOutput;
import javolution.util.FastSet;
import rescuecore2.messages.Command;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.*;
import rescuecore2.standard.messages.AKSpeak;
import rescuecore2.worldmodel.EntityID;
import rescuecore2.worldmodel.Property;

import javax.annotation.Nonnull;
import java.awt.*;
import java.awt.geom.Point2D;
import java.util.List;
import java.util.Queue;
import java.util.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;


public class TIMRADWorldHelper extends AbstractModule {
    protected Human selfHuman;
    protected Building selfBuilding;
    protected EntityID selfId;
    protected AgentInfo agentInfo;
    protected WorldInfo worldInfo;
    protected ScenarioInfo scenarioInfo;
    protected ModuleManager moduleManager;
    protected DevelopData developData;

    protected Set<EntityID> roadsSeen;
    protected Set<EntityID> buildingsSeen;
    protected Set<EntityID> civiliansSeen;
    protected Set<EntityID> fireBrigadesSeen;
    protected Set<EntityID> blockadesSeen;
    protected Set<EntityID> burningBuildings;
    protected Set<EntityID> collapsedBuildings;
    protected Set<EntityID> emptyBuildings;
    protected Set<EntityID> availableHydrants;
    protected Set<EntityID> stuckAgents;

    protected Map<EntityID, TIMRADBlockade> TIMRADBlockadeMap;
    protected Map<EntityID, TIMRADBuilding> TIMRADBuildingMap;
    protected Map<EntityID, TIMRADRoad> TIMRADRoadMap;
    protected Map<EntityID, TIMRADHydrant> TIMRADHydrantMap;
    protected Map<String, Building> buildingXYMap;
    protected Map<EntityID, Boolean> receivedRoads;
    protected Map<EntityID,Integer> receivedCivilianTimeMap;
    protected Map<EntityID, Integer> sendCivilianTimeMap;
    protected int minX, maxX, minY, maxY;
    protected Set<StandardEntity> mapBorderBuildings;
    protected Dimension mapDimension;
    private Area mapCenter;
    protected double mapWidth;
    protected double mapHeight;
    protected double mapDiameter;
    protected boolean isMapHuge = false;
    protected boolean isMapMedium = false;
    protected boolean isMapSmall = false;
    private long uniqueMapNumber;

    protected boolean communicationLess = false;
    protected boolean communicationLow = false;
    protected boolean communicationMedium = false;
    protected boolean communicationHigh = false;
    protected int stayTime;
    protected EntityID lastPosition;
    protected ConfigConstants config;
    protected Map<Property, Integer> propertyTimeMap;

    protected GraphHelper graph;

    private EntityID searchTarget;
    private Map<EntityID, Integer> lastSendTime;
    private Set<EntityID>  agentsPosition;
    private EntityID dominanceAgentID;
    final Class<?>[] standardMessageArgTypes;
    public TIMRADWorldHelper(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
        this.agentInfo = ai;
        this.worldInfo = wi;
        this.scenarioInfo = si;
        this.moduleManager = moduleManager;
        this.developData = developData;
        if (agentInfo.me() instanceof Building) {
            selfBuilding = (Building) agentInfo.me();
        } else {
            selfHuman = (Human) agentInfo.me();
        }
        selfId = agentInfo.me().getID();
        roadsSeen = new HashSet<>();
        buildingsSeen = new HashSet<>();
        civiliansSeen = new HashSet<>();
        fireBrigadesSeen = new HashSet<>();
        blockadesSeen = new HashSet<>();
        burningBuildings = new HashSet<>();
        collapsedBuildings = new HashSet<>();
        emptyBuildings = new HashSet<>();
        availableHydrants = new HashSet<>();
        stuckAgents = new HashSet<>();

        TIMRADBlockadeMap = new HashMap<>();
        TIMRADBuildingMap = new HashMap<>();
        TIMRADRoadMap = new HashMap<>();
        TIMRADHydrantMap = new HashMap<>();
        buildingXYMap = new HashMap<>();

        propertyTimeMap = new HashMap<>();
        lastSendTime = new HashMap<>();

        receivedRoads = new HashMap<>();
        receivedCivilianTimeMap = new HashMap<>();
        sendCivilianTimeMap = new HashMap<>();
        agentsPosition = new HashSet<>();
        dominanceAgentID = new EntityID( 0 );
        lastPosition = new EntityID(0);
        this.standardMessageArgTypes = new Class[]{Boolean.TYPE, Integer.TYPE, Integer.TYPE, BitStreamReader.class};
        config = new ConfigConstants(scenarioInfo.getRawConfig(), this);
        graph = moduleManager.getModule("GraphHelper.Default", TIMRADConstants.GRAPH_HELPER_DEFAULT);


        registerModule(graph);
        initMapInforms();
        initMapCenter();
        initWorldCommunicationCondition();
        initTIMRADBuildings();
        initTIMRADRoads();
        initTIMRADHydrants();
        initTIMRADBlockades();
        initBorderBuildings();
    }

    @Override
    public AbstractModule precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        if (this.getCountPrecompute() >= 2) {
            return this;
        }
        graph.precompute(precomputeData);
        //processVisibilityData(true);
        return this;
    }

    @Override
    public TIMRADWorldHelper resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        if (this.getCountResume() >= 2) {
            return this;
        }
        graph.resume(precomputeData);
        //processVisibilityData(false);
        return this;
    }

    @Override
    public TIMRADWorldHelper preparate() {
        super.preparate();
        if (this.getCountPreparate() >= 2) {
            return this;
        }
        graph.preparate();
        //processVisibilityData(false);
        return this;
    }

    @Override
    public TIMRADWorldHelper updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);
        if (this.getCountUpdateInfo() >= 2) {
            return this;
        }
        graph.updateInfo(messageManager);
        reflectMessage(messageManager);
        roadsSeen.clear();
        buildingsSeen.clear();
        blockadesSeen.clear();
        civiliansSeen.clear();
        fireBrigadesSeen.clear();
        worldInfo.getChanged().getChangedEntities().forEach(changedId -> {
            StandardEntity entity = worldInfo.getEntity(changedId);
            if (entity instanceof Civilian) {
                civiliansSeen.add(entity.getID());
            } else if (entity instanceof Building) {
                Building building = (Building) entity;
                TIMRADBuilding TIMRADBuilding = getTIMRADBuilding(building.getID());
                if (agentInfo.me() instanceof FireBrigade) {
                    if (building.isFierynessDefined() && building.isTemperatureDefined()) {
                        TIMRADBuilding.setEnergy(building.getTemperature() * TIMRADBuilding.getCapacity(), "updateInfo");
                        TIMRADBuilding.updateValues(building);
                    }
                }
                if (building.getFieryness() > 0 && building.getFieryness() < 4) {
                    burningBuildings.add(building.getID());
                } else {
                    burningBuildings.remove(building.getID());
                }

                buildingsSeen.add(building.getID());
                if (building.isOnFire()) {
                    TIMRADBuilding.setIgnitionTime(agentInfo.getTime());
                }
                TIMRADBuilding.setLastSeenTime(agentInfo.getTime());
                TIMRADBuilding.setLastUpdateTime(agentInfo.getTime());
                for (Property p : worldInfo.getChanged().getChangedProperties(building.getID())) {
                    building.getProperty(p.getURN()).takeValue(p);
                    propertyTimeMap.put(p, agentInfo.getTime());
                }

            } else if (entity instanceof Road) {
                Road road = (Road) entity;
                roadsSeen.add(road.getID());

                TIMRADRoad TIMRADRoad = getTIMRADRoad(entity.getID());
                TIMRADRoad.update();
            } else if (entity instanceof Blockade) {
                blockadesSeen.add(entity.getID());
            } else if (entity instanceof FireBrigade) {
                fireBrigadesSeen.add(entity.getID());
            }
        });

        for (TIMRADRoad TIMRADRoad : TIMRADRoadMap.values()) {
            TIMRADRoad.resetPassably();
        }

        this.agentsPosition.clear();
        this.dominanceAgentID = agentInfo.getID();
        for ( StandardEntity entity : worldInfo.getEntitiesOfType( AMBULANCE_TEAM,
                FIRE_BRIGADE, POLICE_FORCE ) ) {
            Human human = (Human) entity;
            this.agentsPosition.add( human.getPosition() );
            if ( agentInfo.getPosition().equals( human.getPosition() )
                    && dominanceAgentID.getValue() > entity.getID().getValue() ) {
                this.dominanceAgentID = entity.getID();
            }
        }
        if(isDominance(agentInfo)){
            sendMessageRoad(messageManager);
            sendMessageCivilian(messageManager);

        }
        if (this.agentInfo.getTime()>this.scenarioInfo.getKernelAgentsIgnoreuntil()){
            sendCommandPolice(messageManager);
        }
        this.lastPosition = agentInfo.getPosition();
        DebugHelper.setGraphEdges(selfId, graph);
        return this;
    }


    @Override
    public TIMRADWorldHelper calc() {
        return null;
    }

    private void initMapInforms() {
        this.minX = Integer.MAX_VALUE;
        this.maxX = Integer.MIN_VALUE;
        this.minY = Integer.MAX_VALUE;
        this.maxY = Integer.MIN_VALUE;
        Pair<Integer, Integer> pos;
        for (StandardEntity standardEntity : worldInfo.getAllEntities()) {
            pos = worldInfo.getLocation(standardEntity);
            if (pos.first() < this.minX)
                this.minX = pos.first();
            if (pos.second() < this.minY)
                this.minY = pos.second();
            if (pos.first() > this.maxX)
                this.maxX = pos.first();
            if (pos.second() > this.maxY)
                this.maxY = pos.second();
        }
        this.mapDimension = new Dimension(maxX - minX, maxY - minY);
        this.mapWidth = mapDimension.getWidth();
        this.mapHeight = mapDimension.getHeight();
        this.mapDiameter = Math.sqrt(Math.pow(this.mapWidth, 2.0) + Math.pow(this.mapHeight, 2.0));
        initMapUniqueNumber();
        initMapSize();
    }

    private void initMapUniqueNumber() {
        long sum = 0;
        for (StandardEntity building : getBuildingsWithURN(worldInfo)) {
            Building b = (Building) building;
            int[] ap = b.getApexList();
            for (int anAp : ap) {
                if (Long.MAX_VALUE - sum <= anAp) {
                    sum = 0;
                }
                sum += anAp;
            }
        }
        uniqueMapNumber = sum;
    }

    private void initMapSize() {
        double mapWidth = this.getMapDimension().getWidth();
        double mapHeight = this.getMapDimension().getHeight();
        double mapDiagonalLength = Math.hypot(mapWidth, mapHeight);
        double rate = mapDiagonalLength / TIMRADConstants.MEAN_VELOCITY_DISTANCE;
        if (rate > 60) {
            this.isMapHuge = true;
        } else if (rate > 30) {
            this.isMapMedium = true;
        } else {
            this.isMapSmall = true;
        }
    }

    protected void initWorldCommunicationCondition() {
        if (scenarioInfo.getCommsChannelsCount() == 1) {
            this.setCommunicationLess(true);
            return;
        }
        int size = 0;
        int maxSize = 0;
        String channelBandwidthKey = "comms.channels.NO.bandwidth";
        for (int i = 1; i < scenarioInfo.getCommsChannelsMaxPlatoon(); i++) {
            size = scenarioInfo.getRawConfig().getIntValue(channelBandwidthKey.replace("NO", String.valueOf(i)));
            maxSize = Math.max(maxSize, size);
        }

        if (size <= 256) {
            this.setCommunicationLow(true);
        } else if (size <= 1024) {
            this.setCommunicationMedium(true);
        } else {
            this.setCommunicationHigh(true);
        }
    }

    private void initTIMRADBuildings() {
        for (StandardEntity entity : getBuildingsWithURN(worldInfo)) {
            TIMRADBuilding TIMRADBuilding;
            Building building = (Building) entity;
            String xy = building.getX() + "," + building.getY();
            buildingXYMap.put(xy, building);
            TIMRADBuilding = new TIMRADBuilding(entity, this);

            if (entity instanceof Refuge || entity instanceof PoliceOffice
                    || entity instanceof FireStation || entity instanceof AmbulanceCentre)
                TIMRADBuilding.setInflammable(false);
            this.TIMRADBuildingMap.put(building.getID(), TIMRADBuilding);
        }

        for (TIMRADBuilding next : TIMRADBuildingMap.values()) {
            Collection<StandardEntity> neighbour = getObjectsInRange(next.getId(), TIMRADWall.MAX_SAMPLE_DISTANCE);

            for (StandardEntity entity : neighbour) {
                if (entity instanceof Building) {
                    next.addNeighbourBuilding(this.TIMRADBuildingMap.get(entity.getID()));
                }
            }
        }
    }

    private void initTIMRADRoads() {
        TIMRADRoad TIMRADRoad;
        Road road;
        for (StandardEntity entity : getRoadsWithURN(worldInfo)) {
            road = (Road) entity;
            TIMRADRoad = new TIMRADRoad(road, this);
            this.TIMRADRoadMap.put(entity.getID(), TIMRADRoad);
        }
    }

    private void initTIMRADHydrants() {
        TIMRADHydrant TIMRADHydrant;
        for (StandardEntity entity : getHydrantsWithURN(worldInfo)) {
            TIMRADHydrant = new TIMRADHydrant(entity.getID());
            this.TIMRADHydrantMap.put(entity.getID(), TIMRADHydrant);
        }
    }

    private void initTIMRADBlockades() {
        TIMRADBlockade TIMRADBlockade;
        for (StandardEntity entity : worldInfo.getEntitiesOfType(StandardEntityURN.BLOCKADE)) {
            TIMRADBlockade = new TIMRADBlockade(entity.getID(), this);
            this.TIMRADBlockadeMap.put(entity.getID(), TIMRADBlockade);
        }
    }

    private void initBorderBuildings() {
        CompositeConvexHull convexHull = new CompositeConvexHull();
        Set<StandardEntity> allEntities = new FastSet<StandardEntity>();
        for (StandardEntity entity : worldInfo.getAllEntities()) {
            if (entity instanceof Building) {
                allEntities.add(entity);
                Pair<Integer, Integer> location = worldInfo.getLocation(entity);
                convexHull.addPoint(location.first(), location.second());
            }
        }
        mapBorderBuildings = PolygonScaler.getMapBorderBuildings(convexHull, allEntities, 0.9);
    }


    private void initMapCenter() {
        double ret;
        int min_x = Integer.MAX_VALUE;
        int max_x = Integer.MIN_VALUE;
        int min_y = Integer.MAX_VALUE;
        int max_y = Integer.MIN_VALUE;

        Collection<StandardEntity> areas = getAreasWithURN(worldInfo);

        long x = 0, y = 0;
        Area result;

        for (StandardEntity entity : areas) {
            Area area1 = (Area) entity;
            x += area1.getX();
            y += area1.getY();
        }

        x /= areas.size();
        y /= areas.size();
        result = (Area) areas.iterator().next();
        for (StandardEntity entity : areas) {
            Area temp = (Area) entity;
            double a = Ruler.getDistance((int) x, (int) y, result.getX(), result.getY());
            double b = Ruler.getDistance((int) x, (int) y, temp.getX(), temp.getY());
            if (a > b) {
                result = temp;
            }

            if (temp.getX() < min_x) {
                min_x = temp.getX();
            } else if (temp.getX() > max_x)
                max_x = temp.getX();

            if (temp.getY() < min_y) {
                min_y = temp.getY();
            } else if (temp.getY() > max_y)
                max_y = temp.getY();
        }
        ret = (Math.pow((min_x - max_x), 2) + Math.pow((min_y - max_y), 2));
        ret = Math.sqrt(ret);
        this.mapCenter = result;
    }

    private void reflectMessage(MessageManager messageManager) {
        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        changedEntities.add(this.agentInfo.getID());
        for (CommunicationMessage message : messageManager.getReceivedMessageList()) {
            if (message instanceof MessageBuilding) {

                MessageBuilding mb = (MessageBuilding) message;
                if (!changedEntities.contains(mb.getBuildingID())) {
                    MessageUtil.reflectMessage(this.worldInfo, mb);
                    if (agentInfo.me() instanceof FireBrigade) {
                        updateBuildingFuelForFireBrigade(getEntity(mb.getBuildingID(), Building.class));
                        int receivedTime;
                        if (mb.isRadio()) {
                            receivedTime = agentInfo.getTime() - 1;
                        } else {
                            receivedTime = agentInfo.getTime() - 5;
                        }
                        Building building = (Building) this.getEntity(mb.getBuildingID());
                        if (getPropertyTime(building.getFierynessProperty()) < receivedTime) {
                            propertyTimeMap.put(building.getFierynessProperty(), receivedTime);
                            propertyTimeMap.put(building.getTemperatureProperty(), receivedTime);
                        }
                        TIMRADBuilding TIMRADBuilding = getTIMRADBuilding(mb.getBuildingID());
                        TIMRADBuilding.setLastUpdateTime(receivedTime);
                    }
                }
            } else if (message instanceof MessageRoad) {


                MessageRoad mr = (MessageRoad) message;
                MessageUtil.reflectMessage(this.worldInfo, mr);
                Boolean passable = mr.isPassable();
                if ( passable != null ) {
                    this.receivedRoads.put(mr.getRoadID(),passable);
                }

                if (mr.isPassable()) {
                    List<MyEdge> myEdgesInArea = graph.getMyEdgesInArea(mr.getRoadID());
                    for (MyEdge edge : myEdgesInArea) {
                        edge.setPassable(true);
                    }
                }
                if (!changedEntities.contains(mr.getRoadID()) && !mr.isPassable() && !mr.isBlockadeDefined() &&
                        !mr.getSenderID().equals(agentInfo.getID())) {
                    List<MyEdge> myEdgesInArea = graph.getMyEdgesInArea(mr.getRoadID());
                    for (MyEdge myEdge : myEdgesInArea) {
                        myEdge.setPassable(false);
                    }
                }
                TIMRADRoadMap.get(mr.getRoadID()).update();
            }else if(message instanceof MessageCivilian){
                StandardEntity entity = MessageUtil.reflectMessage( worldInfo, (StandardMessage) message );
                if (entity instanceof Civilian){
                    this.receivedCivilianTimeMap.put( entity.getID(), agentInfo.getTime() );
                }
            }
        }
        DebugHelper.setGraphEdges(selfId, graph);
    }

    private void sendCommandPolice(MessageManager messageManager){
        EntityID nowPosition = this.agentInfo.getPosition();
        StandardEntity standardEntity = this.worldInfo.getEntity(nowPosition);
        Human human = (Human) this.agentInfo.me();

        if (standardEntity instanceof Building&&this.hasBlockades(standardEntity)&&this.isStuckInBuilding()){
            this.stayTime = 0;
            messageManager.addMessage( new CommandPolice( true, StandardMessagePriority.HIGH,null,
                    this.agentInfo.getPosition(), CommandPolice.ACTION_CLEAR ) );
        }
    }
    private Boolean hasBlockades(StandardEntity target){
        Queue<StandardEntity> visited = new LinkedList<>();;
        Queue<StandardEntity> open_list = new LinkedList<>();;
        open_list.add(target);
        while(!open_list.isEmpty()){
            StandardEntity standardEntity = open_list.poll();
            Building building = (Building) standardEntity;
            visited.add(standardEntity);
            for (EntityID neighbourID : building.getNeighbours()) {
                StandardEntity neighbour = this.worldInfo.getEntity(neighbourID);
                if (neighbour instanceof Road || neighbour instanceof Hydrant) {
                    Road road = (Road) neighbour;
                    if (road.isBlockadesDefined()&&(road.getBlockades()!=null&&!road.getBlockades().isEmpty())){
                        return true;
                    }
                }
                else if(neighbour instanceof Building)
                {
                    if(!visited.contains(neighbour))
                    {
                        open_list.add(neighbour);
                    }
                }
            }
        }
        return false;
    }

    private Boolean isStuckInBuilding(){
        //
        if (agentInfo.me() instanceof PoliceForce){
            PoliceForce policeForce = (PoliceForce) agentInfo.me();
            if (!(policeForce.isBuriednessDefined()&&policeForce.getBuriedness()>0)){
                return false;
            }
        }
        EntityID nowPosition = this.agentInfo.getPosition();
        StandardEntity position = this.worldInfo.getEntity(nowPosition);
        Queue<StandardEntity> visited = new LinkedList<>();;
        Queue<StandardEntity> open_list = new LinkedList<>();;
        open_list.add(position);
        Boolean flag = true;
        while(!open_list.isEmpty()){
            StandardEntity standardEntity = open_list.poll();
            Building building = (Building) standardEntity;
            visited.add(standardEntity);
            for (EntityID neighbourID : building.getNeighbours()) {
                StandardEntity neighbour = this.worldInfo.getEntity(neighbourID);
                if (neighbour instanceof Road || neighbour instanceof Hydrant) {
                    Road road = (Road) neighbour;
                    TIMRADRoad TIMRADRoad = this.getTIMRADRoad(road.getID());
                    if (TIMRADRoad.isPassable()){
                        flag = false;
                    }
                }
                else if(neighbour instanceof Building)
                {
                    if(!visited.contains(neighbour))
                    {
                        open_list.add(neighbour);
                    }
                }
            }
        }
        if (flag){
            return true;
        }
        if (agentInfo.me() instanceof FireBrigade || agentInfo.me() instanceof AmbulanceTeam){
            Collection<EntityID> changed = this.agentInfo.getChanged().getChangedEntities();
            for (EntityID entityID : changed){
                StandardEntity standardEntity = this.worldInfo.getEntity(entityID);
                if (standardEntity instanceof Human ){
                    Human human =(Human) standardEntity;
                    if (human.getPosition().equals(this.agentInfo.getPosition())&&human.getHP()>0&&human.getBuriedness()>0){
                        return false;
                    }
                }
            }
        }
        StandardEntity standardEntity = this.worldInfo.getEntity(nowPosition);
        Human human = (Human) this.agentInfo.me();
        if (human.isBuriednessDefined()&&human.getBuriedness()>0){
            return false;
        }
        if (standardEntity instanceof Building){
            if (lastPosition.equals(nowPosition)){
                this.stayTime++;
            }else {
                this.stayTime = 0;
            }
        }
        if (this.stayTime>3){
            return true;
        }else {
            return false;
        }
    }
    private void sendMessageRoad(MessageManager messageManager) {
        Set<EntityID> targetRoad = new HashSet<>();
        Collection<Command> heard = this.agentInfo.getHeard();
        Set<EntityID> lastSenderRoad = new HashSet<>();
        for (Command command : heard){
            if (command instanceof AKSpeak && ((AKSpeak)command).getAgentID().getValue()==this.agentInfo.getID().getValue()){
                AKSpeak received = (AKSpeak)command;
                byte[] receivedData = received.getContent();
                boolean isRadio = received.getChannel() != 0;
                if (isRadio && receivedData.length >0){
                    MessageRoad messageRoad= this.getMessageRoad(messageManager, Boolean.TRUE, received.getAgentID(), receivedData);
                    if (messageRoad != null){
                        lastSenderRoad.add(messageRoad.getRoadID());
                    }
                }
            }
        }
        for(EntityID entityID : this.lastSendTime.keySet()){
            if(this.lastSendTime.get(entityID) == this.agentInfo.getTime()-1){
                Road road = (Road) this.worldInfo.getEntity(entityID);
                if (!lastSenderRoad.contains(entityID)){
                    targetRoad.add(entityID);
                }
            }
        }
        targetRoad.addAll(roadsSeen);
        for (EntityID id : targetRoad) {
            Road road = (Road) worldInfo.getEntity(id);
            List<MyEdge> myEdgesInArea = graph.getMyEdgesInArea(id);
            boolean passable = false;
            for (MyEdge myEdge : myEdgesInArea) {
                if (myEdge.isPassable()) {
                    passable = true;
                    break;
                }
            }
            if (!passable) {

                if (!lastSendTime.containsKey(id) || agentInfo.getTime() - lastSendTime.get(id) > 5) {
                    messageManager.addMessage(new MessageRoad(true, road, null, false, false));
                    lastSendTime.put(id, agentInfo.getTime());

                }
            }else {
                if (this.agentInfo.me() instanceof PoliceForce) {
//                    messageManager.addMessage(new MessageRoad(false, road, null, true, false));
                    if (!(this.receivedRoads.containsKey(id)&&this.receivedRoads.get(id))){
                        messageManager.addMessage(new MessageRoad(true, road, null, true, false));
                        lastSendTime.put(id, agentInfo.getTime());
                        this.receivedRoads.put(id,true);
                    }
                }
            }
        }

    }

    public void sendMessageCivilian(MessageManager messageManager){
        for (EntityID entityID : this.civiliansSeen){
            Civilian civilian = (Civilian) worldInfo.getEntity(entityID);
            if (civilian.isPositionDefined()){
                StandardEntity position = worldInfo.getEntity(civilian.getPosition());
                if (! (position instanceof AmbulanceTeam) && !(position instanceof Refuge) && position instanceof Building){
                    Building building = (Building) position;
                    if (building.isBrokennessDefined() && building.getBrokenness()>0){
                        messageManager.addMessage(new MessageCivilian(true, StandardMessagePriority.HIGH,civilian));
                        this.sendCivilianTimeMap.put(entityID,agentInfo.getTime());
                    }
                }
                if (position instanceof AmbulanceTeam){
                    messageManager.addMessage(new MessageCivilian(true, StandardMessagePriority.NORMAL,civilian));
                    this.sendCivilianTimeMap.put(entityID,agentInfo.getTime());
                }
            }
        }

        Collection<Command> heard = this.agentInfo.getHeard();
        Set<EntityID> lastSenderCivilian = new HashSet<>();
        for (Command command : heard){
            if (command instanceof AKSpeak && ((AKSpeak)command).getAgentID().getValue()==this.agentInfo.getID().getValue()){
                AKSpeak received = (AKSpeak)command;
                byte[] receivedData = received.getContent();
                boolean isRadio = received.getChannel() != 0;
                if (isRadio && receivedData.length >0){
                    MessageCivilian messageCivilian = this.getMessageCivilian(messageManager, Boolean.TRUE, received.getAgentID(), receivedData);
                    if (messageCivilian != null){
                        lastSenderCivilian.add(messageCivilian.getAgentID());
                    }
                }
            }
        }
        for(EntityID entityID : this.sendCivilianTimeMap.keySet()){
            if(this.sendCivilianTimeMap.get(entityID) == this.agentInfo.getTime()-1){
                Civilian civilian = (Civilian) this.worldInfo.getEntity(entityID);
                if (!lastSenderCivilian.contains(entityID)){
                    messageManager.addMessage(new MessageCivilian(true, StandardMessagePriority.HIGH,civilian));
                }
            }
        }
    }

    private MessageCivilian getMessageCivilian(@Nonnull MessageManager messageManager, boolean isRadio, @Nonnull EntityID senderID, byte[] data) {
        MessageCivilian messageCivilian = null;
        BitStreamReader bitStreamReader = new BitStreamReader(data);
        int messageClassIndex = bitStreamReader.getBits(5);
        if (messageClassIndex <= 0) {
            ConsoleOutput.out(ConsoleOutput.State.WARN, "ignore Message Class Index (0)");
        } else {
            int messageTTL = isRadio ? -1 : bitStreamReader.getBits(3);
            Object[] args = new Object[]{isRadio, senderID.getValue(), messageTTL, bitStreamReader};

            try {
                CommunicationMessage cm = (CommunicationMessage)messageManager.getMessageClass(messageClassIndex).getConstructor(this.standardMessageArgTypes).newInstance(args);
                if (cm instanceof MessageCivilian){
                    messageCivilian = (MessageCivilian) cm;
                }
            } catch (IllegalArgumentException | NoSuchMethodException var10) {
                var10.printStackTrace();
            } catch (ReflectiveOperationException var11) {
                var11.printStackTrace();
            }

        }
        return messageCivilian;
    }

    private MessageRoad getMessageRoad(@Nonnull MessageManager messageManager, boolean isRadio, @Nonnull EntityID senderID, byte[] data) {
        MessageRoad messageRoad = null;
        BitStreamReader bitStreamReader = new BitStreamReader(data);
        int messageClassIndex = bitStreamReader.getBits(5);
        if (messageClassIndex <= 0) {
            ConsoleOutput.out(ConsoleOutput.State.WARN, "ignore Message Class Index (0)");
        } else {
            int messageTTL = isRadio ? -1 : bitStreamReader.getBits(3);
            Object[] args = new Object[]{isRadio, senderID.getValue(), messageTTL, bitStreamReader};

            try {
                CommunicationMessage cm = (CommunicationMessage)messageManager.getMessageClass(messageClassIndex).getConstructor(this.standardMessageArgTypes).newInstance(args);
                if (cm instanceof MessageRoad){
                    messageRoad = (MessageRoad) cm;
                }
            } catch (IllegalArgumentException | NoSuchMethodException var10) {
                var10.printStackTrace();
            } catch (ReflectiveOperationException var11) {
                var11.printStackTrace();
            }

        }
        return messageRoad;
    }

    public boolean isDominance( AgentInfo agentInfo ) {
        return agentInfo.getID().equals( this.dominanceAgentID );
    }

    private boolean isRecentlySendCivilian( AgentInfo agentInfo, EntityID id ) {
        return ( this.sendCivilianTimeMap.containsKey( id ) && ( ( agentInfo.getTime()
                - this.sendCivilianTimeMap.get( id ) ) < 5) );
    }

    private boolean isRecentlyReceivedCivilian( AgentInfo agentInfo, EntityID id ) {
        return ( this.receivedCivilianTimeMap.containsKey( id ) && ( ( agentInfo.getTime()
                - this.receivedCivilianTimeMap.get( id ) ) < 5) );
    }

    private void updateBuildingFuelForFireBrigade(Building building) {
        TIMRADBuilding TIMRADBuilding = this.getTIMRADBuilding(building.getID());
        TIMRADBuilding.setVisible(true);
        if (building.isFierynessDefined() && building.isTemperatureDefined()) {
            int temperature = building.getTemperature();
            TIMRADBuilding.setEnergy(temperature * TIMRADBuilding.getCapacity(), "updateBuildingFuelForFireBrigade");
            switch (building.getFieryness()) {
                case 0:
                TIMRADBuilding.setFuel(TIMRADBuilding.getInitialFuel());
                    if (TIMRADBuilding.getEstimatedTemperature() >= TIMRADBuilding.getIgnitionPoint()) {
                        TIMRADBuilding.setEnergy(TIMRADBuilding.getIgnitionPoint() / 2, "updateBuildingFuelForFireBrigade");
                    }
                    break;
                case 1:
                    if (TIMRADBuilding.getFuel() < TIMRADBuilding.getInitialFuel() * 0.66) {
                        TIMRADBuilding.setFuel((float) (TIMRADBuilding.getInitialFuel() * 0.75));
                    } else if (TIMRADBuilding.getFuel() == TIMRADBuilding.getInitialFuel()) {
                        TIMRADBuilding.setFuel((float) (TIMRADBuilding.getInitialFuel() * 0.90));
                    }
                    break;
                case 2:
                    if (TIMRADBuilding.getFuel() < TIMRADBuilding.getInitialFuel() * 0.33
                            || TIMRADBuilding.getFuel() > TIMRADBuilding.getInitialFuel() * 0.66) {
                                TIMRADBuilding.setFuel((float) (TIMRADBuilding.getInitialFuel() * 0.50));
                    }
                    break;
                case 3:
                    if (TIMRADBuilding.getFuel() < TIMRADBuilding.getInitialFuel() * 0.01
                            || TIMRADBuilding.getFuel() > TIMRADBuilding.getInitialFuel() * 0.33) {
                        TIMRADBuilding.setFuel((float) (TIMRADBuilding.getInitialFuel() * 0.15));
                    }
                    break;
                case 8:
                TIMRADBuilding.setFuel(0);
                    break;
                default:
                    break;
            }
        }
    }

    public <T extends StandardEntity> List<T> getEntitiesOfType(Class<T> c, StandardEntityURN urn) {
        Collection<StandardEntity> entities = worldInfo.getEntitiesOfType(urn);
        List<T> list = new ArrayList<T>();
        for (StandardEntity entity : entities) {
            if (c.isInstance(entity)) {
                list.add(c.cast(entity));
            }
        }
        return list;
    }

    public Collection<StandardEntity> getEntitiesOfType(EnumSet<StandardEntityURN> urns) {
        Collection<StandardEntity> res = new HashSet<StandardEntity>();
        for (StandardEntityURN urn : urns) {
            res.addAll(worldInfo.getEntitiesOfType(urn));
        }
        return res;
    }

    public <T extends StandardEntity> T getEntity(EntityID id, Class<T> c) {
        StandardEntity entity;
        entity = worldInfo.getEntity(id);
        if (c.isInstance(entity)) {
            T castedEntity = c.cast(entity);

            return castedEntity;
        } else {
            return null;
        }
    }

    public StandardEntity getEntity(EntityID id) {
        return worldInfo.getEntity(id);
    }

    public TIMRADBuilding getTIMRADBuilding(StandardEntity entity) {
        return this.TIMRADBuildingMap.get(entity.getID());
    }

    public TIMRADBuilding getTIMRADBuilding(EntityID entityId) {
        return this.TIMRADBuildingMap.get(entityId);
    }

    public TIMRADRoad getTIMRADRoad(StandardEntity entity) {
        return this.TIMRADRoadMap.get(entity.getID());
    }

    public TIMRADRoad getTIMRADRoad(EntityID entityId) {
        return this.TIMRADRoadMap.get(entityId);
    }

    public Collection<TIMRADBuilding> getTIMRADBuildings() {
        return this.TIMRADBuildingMap.values();
    }

    public Map<EntityID, TIMRADBuilding> getTIMRADBuildingMap() {
        return TIMRADBuildingMap;
    }

    public Map<EntityID, TIMRADRoad> getTIMRADRoadMap() {
        return TIMRADRoadMap;
    }

    public TIMRADHydrant getTIMRADHydrant(EntityID entityid) {
        return this.TIMRADHydrantMap.get(entityid);
    }

    /* ---------------------------------------------------------------------------------------------------------- */

    public boolean isMapHuge() {
        return this.isMapHuge;
    }

    public boolean isMapMedium() {
        return this.isMapMedium;
    }

    public boolean isMapSmall() {
        return this.isMapSmall;
    }

    public Point getMapCenterPoint() {    ///should be >> 1
        return new Point((int) mapWidth >> 2, (int) mapHeight >> 2);
    }

    public List<Building> findNearBuildings(Building centerbuilding, int distance) {
        List<Building> result;
        Collection<StandardEntity> allObjects;
        int radius;

        Rectangle rect = centerbuilding.getShape().getBounds();
        radius = (int) (distance + rect.getWidth() + rect.getHeight());

        allObjects = worldInfo.getObjectsInRange(centerbuilding, radius);
        result = new ArrayList<Building>();
        for (StandardEntity next : allObjects) {
            if (next instanceof Building) {
                Building building;

                building = (Building) next;
                if (!building.equals(centerbuilding)) {
                    if (Ruler.getDistance(centerbuilding, building) < distance) {
                        result.add(building);
                    }
                }
            }
        }
        return result;
    }

    public int distance(Point p1, Point p2) {
        return distance(p1.x, p1.y, p2.x, p2.y);
    }

    public int distance(int x1, int y1, int x2, int y2) {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return (int) Math.hypot(dx, dy);
    }

    public Line getLine(Area node1, Area node2) {
        int x1 = node1.getX();
        int y1 = node1.getY();
        int x2 = node2.getX();
        int y2 = node2.getY();
        return new Line(x1, y1, x2, y2);
    }

    public Area getPositionFromCoordinates(int x, int y) {
        for (StandardEntity entity : getObjectsInRange(x, y, 1000)) {
            if (entity instanceof Area) {
                Area area = (Area) entity;
                if (area.getShape().contains(x, y)) {
                    return area;
                }
            }
        }
        return null;
    }

    public StandardEntity getPositionFromCoordinates(Pair<Integer, Integer> coordinate) {
        return getPositionFromCoordinates(coordinate.first(), coordinate.second());
    }

    public Set<StandardEntity> getNeighbours(StandardEntity e, int Ext) {
        Set<StandardEntity> Neighbours = new HashSet<StandardEntity>();
        if (e instanceof Building) {
            for (EntityID tmp2 : ((Building) e).getNeighbours()) {
                Neighbours.add(worldInfo.getEntity(tmp2));
            }
            if (((Building) e).isEdgesDefined()) {
                Ext *= 1000;
                List<Edge> Edges = new ArrayList<Edge>();
                Edges = ((Building) e).getEdges();
                Polygon ExtArea = new Polygon();
                Polygon baseArea = new Polygon();
                int n = 1;
                Point2D.Double tmp1 = new Point2D.Double();
                Point2D.Double tmp2 = new Point2D.Double();
                Point2D.Double tmp3 = new Point2D.Double();
                Point2D.Double tmp4 = new Point2D.Double();
                Point2D.Double tmp5 = new Point2D.Double();
                for (Edge Ee : Edges) {
                    baseArea.addPoint(Ee.getStartX(), Ee.getStartY());
                }
                for (Edge Ee : Edges) {
                    for (Edge Ee1 : Edges) {
                        if (Ee.getStart().equals(Ee1.getEnd())) {
                            tmp1.setLocation
                                    (
                                            Ee.getStartX() + (n * (Ee.getEndX() - Ee.getStartX())) / Math.hypot((double) Ee.getEndX() - (double) Ee.getStartX(), (double) Ee.getEndY() - (double) Ee.getStartY())
                                            ,
                                            Ee.getStartY() + (n * (Ee.getEndY() - Ee.getStartY())) / Math.hypot((double) Ee.getEndX() - (double) Ee.getStartX(), (double) Ee.getEndY() - (double) Ee.getStartY())
                                    );
                            tmp2.setLocation
                                    (
                                            Ee1.getEndX() + (n * (Ee1.getStartX() - Ee1.getEndX())) / Math.hypot((double) Ee1.getStartX() - (double) Ee1.getEndX(), (double) Ee1.getStartY() - (double) Ee1.getEndY())
                                            ,
                                            Ee1.getEndY() + (n * (Ee1.getStartY() - Ee1.getEndY())) / Math.hypot((double) Ee1.getStartX() - (double) Ee1.getEndX(), (double) Ee1.getStartY() - (double) Ee1.getEndY())
                                    );

                            tmp3.setLocation
                                    (
                                            (tmp1.x + tmp2.x) / 2
                                            ,
                                            (tmp1.y + tmp2.y) / 2
                                    );
                            tmp4.setLocation(Ee.getStartX(), Ee.getStartY());
                            if (tmp3.x == tmp4.x && tmp3.y == tmp4.y) {
                                continue;
                            } else if (baseArea.contains(tmp3)) {
                                tmp5.setLocation(
                                        tmp4.x + (Ext * (tmp4.x - tmp3.x) / Math.hypot(tmp4.x - tmp3.x, tmp4.y - tmp3.y))
                                        ,
                                        tmp4.y + (Ext * (tmp4.y - tmp3.y) / Math.hypot(tmp4.x - tmp3.x, tmp4.y - tmp3.y)));

                            } else if (!baseArea.contains(tmp3)) {
                                tmp5.setLocation(
                                        tmp4.x + (Ext * (tmp3.x - tmp4.x) / Math.hypot(tmp3.x - tmp4.x, tmp3.y - tmp4.y))
                                        ,
                                        tmp4.y + (Ext * (tmp3.y - tmp4.y) / Math.hypot(tmp3.x - tmp4.x, tmp3.y - tmp4.y)));
                            }
                            ExtArea.addPoint((int) tmp5.x, (int) tmp5.y);
                        }
                    }
                }
                for (StandardEntity checker : worldInfo.getObjectsInRange(e, 100 * 1000)) {
                    if (checker instanceof Building) {
                        if (((Building) checker).isEdgesDefined()) {
                            for (Edge Edger : ((Building) checker).getEdges()) {
                                if (ExtArea.contains(Edger.getStartX(), Edger.getStartY())) {
                                    Neighbours.add(checker);
                                } else if (ExtArea.contains((Edger.getStartX() + Edger.getEndX()) / 2, (Edger.getStartY() + Edger.getEndY()) / 2)) {
                                    Neighbours.add(checker);
                                }
                            }
                        }
                    }
                }
            }
        } else if (e instanceof Road) {
            for (EntityID Neighs : ((Road) e).getNeighbours()) {
                if (worldInfo.getEntity(Neighs) instanceof Road) {
                    Neighbours.add(worldInfo.getEntity(Neighs));
                } else if (worldInfo.getEntity(Neighs) instanceof Building) {
                    Neighbours.add(worldInfo.getEntity(Neighs));
                }
            }
        }
        return Neighbours;
    }

    public int getDistance(EntityID first, EntityID second) {
        return worldInfo.getDistance(first, second);
    }

    public int getDistance(StandardEntity first, StandardEntity second) {
        return worldInfo.getDistance(first, second);
    }

    public Collection<StandardEntity> getObjectsInRange(EntityID entityID, int distance) {
        return worldInfo.getObjectsInRange(entityID, distance);
    }

    public Collection<StandardEntity> getObjectsInRange(int x, int y, int range) {
        return worldInfo.getObjectsInRange(x, y, range);
    }

    public Collection<Blockade> getBlockadesInRange(int range) {
        Collection<Blockade> result = new HashSet<>();
        Collection<StandardEntity> objectsInRange = worldInfo.getObjectsInRange(agentInfo.getID(), range);
        for (StandardEntity entity : objectsInRange) {
            if (entity instanceof Road) {
                TIMRADRoad TIMRADRoad = TIMRADRoadMap.get(entity.getID());
                List<TIMRADBlockade> TIMRADBlockades = TIMRADRoad.getTIMRADBlockades();
                for (TIMRADBlockade TIMRADBlockade : TIMRADBlockades) {
                    if (Ruler.getDistance(TIMRADBlockade.getPolygon(), getSelfLocation()) <= range) {
                        result.add(TIMRADBlockade.getSelfBlockade());
                    }
                }
            }
        }
        return result;
    }

    public Collection<TIMRADRoad> getNeighborRoads() {
        HashSet<TIMRADRoad> results = new HashSet<>();
        Area selfArea = (Area) getSelfPosition();
        List<EntityID> neighbours = selfArea.getNeighbours();
        for (EntityID id : neighbours) {
            TIMRADRoad TIMRADRoad = TIMRADRoadMap.get(id);
            if (TIMRADRoad != null) {
                results.add(TIMRADRoad);
            }
        }
        return results;
    }

    public TIMRADRoad getNearestNeighborRoad() {
        Collection<TIMRADRoad> neighborRoads = getNeighborRoads();
        double minDistance = Double.MAX_VALUE;
        TIMRADRoad nearestRoad = null;
        for (TIMRADRoad road : neighborRoads) {
            Polygon polygon = road.getPolygon();
            double distance = Ruler.getDistance(polygon, getSelfLocation());
            if (distance < minDistance) {
                minDistance = distance;
                nearestRoad = road;
            }
        }
        return nearestRoad;
    }

    public Set<EntityID> getBurningBuildings() {
        return burningBuildings;
    }

    public Set<EntityID> getCollapsedBuildings() {
        return collapsedBuildings;
    }

    public StandardEntity getSelfPosition() {
        if (worldInfo.getEntity(agentInfo.getID()) instanceof Building) {
            return selfBuilding;
        } else {
            return agentInfo.getPositionArea();
        }
    }

    public EntityID getSelfPositionId() {
        return getSelfPosition().getID();
    }

    public Pair<Integer, Integer> getSelfLocation() {
        return worldInfo.getLocation(agentInfo.me());
    }

    public Human getSelfHuman() {
        return selfHuman;
    }

    public Building getSelfBuilding() {
        return selfBuilding;
    }

    public Pair<Integer, Integer> getLocation(StandardEntity entity) {
        return worldInfo.getLocation(entity);
    }

    public Pair<Integer, Integer> getLocation(EntityID entityID) {
        return worldInfo.getLocation(entityID);
    }

    public int getTime() {
        return agentInfo.getTime();
    }

    public Set<StandardEntity> getBorderBuildings() {
        return this.mapBorderBuildings;
    }

    public Dimension getMapDimension() {
        return this.mapDimension;
    }

    public double getMapWidth() {
        return this.mapWidth;
    }

    public double getMapHeight() {
        return this.mapHeight;
    }

    public boolean isCommunicationLess() {
        return this.communicationLess;
    }

    public void setCommunicationLess(boolean communicationLess) {
        this.communicationLess = communicationLess;
    }

    public boolean isCommunicationLow() {
        return this.communicationLow;
    }

    public void setCommunicationLow(boolean communicationLow) {
        this.communicationLow = communicationLow;
    }

    public boolean isCommunicationMedium() {
        return this.communicationMedium;
    }

    public void setCommunicationMedium(boolean communicationMedium) {
        this.communicationMedium = communicationMedium;
    }

    public boolean isCommunicationHigh() {
        return this.communicationHigh;
    }

    public void setCommunicationHigh(boolean communicationHigh) {
        this.communicationHigh = communicationHigh;
    }

    public Set<EntityID> getStuckedAgents() {
        return this.stuckAgents;
    }

    public int getMinX() {
        return minX;
    }

    public int getMaxX() {
        return maxX;
    }

    public int getMinY() {
        return minY;
    }

    public int getMaxY() {
        return maxY;
    }

    public AgentInfo getAgentInfo() {
        return agentInfo;
    }

    public WorldInfo getWorldInfo() {
        return worldInfo;
    }

    public ScenarioInfo getScenarioInfo() {
        return scenarioInfo;
    }

    public ConfigConstants getConfig() {
        return config;
    }

    public GraphHelper getGraph() {
        return graph;
    }

    public Set<EntityID> getRoadsSeen() {
        return roadsSeen;
    }

    public Set<EntityID> getBuildingsSeen() {
        return buildingsSeen;
    }

    public Set<EntityID> getCiviliansSeen() {
        return civiliansSeen;
    }

    public Set<EntityID> getFireBrigadesSeen() {
        return fireBrigadesSeen;
    }

    public Set<EntityID> getBlockadesSeen() {
        return blockadesSeen;
    }

    public static Collection<StandardEntity> getBuildingsWithURN(WorldInfo worldInfo) {
        return worldInfo.getEntitiesOfType(
                StandardEntityURN.BUILDING,
                StandardEntityURN.REFUGE,
                StandardEntityURN.AMBULANCE_CENTRE,
                StandardEntityURN.POLICE_OFFICE,
                StandardEntityURN.FIRE_STATION,
                StandardEntityURN.GAS_STATION);
    }

    public static Collection<StandardEntity> getHydrantsWithURN(WorldInfo worldInfo) {
        return worldInfo.getEntitiesOfType(StandardEntityURN.HYDRANT);
    }

    public static Collection<StandardEntity> getGasStationsWithUrn(WorldInfo worldInfo) {
        return worldInfo.getEntitiesOfType(StandardEntityURN.GAS_STATION);
    }

    public static Collection<StandardEntity> getRefugesWithUrn(WorldInfo worldInfo) {
        return worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE);
    }

    public static Collection<StandardEntity> getAreasWithURN(WorldInfo worldInfo) {
        return worldInfo.getEntitiesOfType(
                StandardEntityURN.BUILDING,
                StandardEntityURN.REFUGE,
                StandardEntityURN.ROAD,
                StandardEntityURN.AMBULANCE_CENTRE,
                StandardEntityURN.POLICE_OFFICE,
                StandardEntityURN.FIRE_STATION,
                StandardEntityURN.HYDRANT,
                StandardEntityURN.GAS_STATION);
    }

    public static Collection<StandardEntity> getHumansWithURN(WorldInfo worldInfo) {
        return worldInfo.getEntitiesOfType(
                StandardEntityURN.CIVILIAN,
                StandardEntityURN.FIRE_BRIGADE,
                StandardEntityURN.POLICE_FORCE,
                StandardEntityURN.AMBULANCE_TEAM);
    }

    public static Collection<StandardEntity> getAgentsWithURN(WorldInfo worldInfo) {
        return worldInfo.getEntitiesOfType(
                StandardEntityURN.FIRE_BRIGADE,
                StandardEntityURN.POLICE_FORCE,
                StandardEntityURN.AMBULANCE_TEAM,
                StandardEntityURN.FIRE_STATION,
                StandardEntityURN.POLICE_OFFICE,
                StandardEntityURN.AMBULANCE_CENTRE);
    }

    public static Collection<StandardEntity> getPlatoonAgentsWithURN(WorldInfo worldInfo) {
        return worldInfo.getEntitiesOfType(
                StandardEntityURN.FIRE_BRIGADE,
                StandardEntityURN.POLICE_FORCE,
                StandardEntityURN.AMBULANCE_TEAM);
    }

    public static Collection<StandardEntity> getRoadsWithURN(WorldInfo worldInfo) {
        return worldInfo.getEntitiesOfType(StandardEntityURN.ROAD, StandardEntityURN.HYDRANT);
    }

    public List<StandardEntity> getEntities(Set<EntityID> entityIDs) {
        List<StandardEntity> result = new ArrayList<StandardEntity>();
        for (EntityID next : entityIDs) {
            result.add(getEntity(next));
        }
        return result;
    }

    public List<StandardEntity> getEntities(List<EntityID> entityIDs) {
        List<StandardEntity> result = new ArrayList<StandardEntity>();
        for (EntityID next : entityIDs) {
            result.add(getEntity(next));
        }
        return result;
    }

//    public void processVisibilityData(boolean isPrecompute) {
//
//        String visibleFromFileName = CSUConstants.PRECOMPUTE_DIRECTORY + this.getUniqueMapNumber() + ".vif";
//        String observableAreasFileName = CSUConstants.PRECOMPUTE_DIRECTORY + this.getUniqueMapNumber() + ".oba";
//
//        if (new File(visibleFromFileName).exists() && new File(observableAreasFileName).exists() &&
//                !isPrecompute) {
//            Thread loader = new Thread() {
//                @Override
//                public void run() {
//                    loadVisibilityData(visibleFromFileName, observableAreasFileName);
//                }
//            };
//            loader.start();
//        } else {
//            FileEntityIDMap visibleFrom = new FileEntityIDMap();
//            FileEntityIDMap observableAreas = new FileEntityIDMap();
//
//            CSULineOfSightPerception lineOfSightPerception = new CSULineOfSightPerception(this);
//
//
//            Collection<CSURoad> allRoads = this.getCSURoads();
//            Collection<CSUBuilding> allBuildings = this.getCSUBuildings();
//
//            for (CSURoad road : allRoads) {
//                road.setObservableAreas(lineOfSightPerception.getVisibleAreas(road.getId()));
//                observableAreas.put(road.getId().getValue(), Util.fetchIdValueFromElementIds(road.getObservableAreas()));
//            }
//            for (CSUBuilding building : allBuildings) {
//                building.setObservableAreas(lineOfSightPerception.getVisibleAreas(building.getId()));
//                observableAreas.put(building.getId().getValue(), Util.fetchIdValueFromElementIds(building.getObservableAreas()));
//            }
//
//            for (CSURoad road1 : allRoads) {
//                for (CSURoad road2 : allRoads) {
//                    if (road1.equals(road2)) continue;
//                    if (road2.getObservableAreas().contains(road1.getId())) {
//                        road1.getVisibleFrom().add(road2.getId());
//
//                    }
//                }
//                for (CSUBuilding building : allBuildings) {
//                    if (building.getObservableAreas().contains(road1.getId())) {
//                        road1.getVisibleFrom().add(building.getId());
//                    }
//                }
//                visibleFrom.put(road1.getId().getValue(), Util.fetchIdValueFromElementIds(road1.getVisibleFrom()));
//            }
//            for (CSUBuilding building1 : allBuildings) {
//                for (CSUBuilding building2 : allBuildings) {
//                    if (building1.equals(building2)) continue;
//                    if (building2.getObservableAreas().contains(building1.getId())) {
//                        building1.getVisibleFrom().add(building2.getId());
//
//                    }
//                }
//                for (CSURoad road : allRoads) {
//                    if (road.getObservableAreas().contains(building1.getId())) {
//                        building1.getVisibleFrom().add(road.getId());
//                    }
//                }
//                visibleFrom.put(building1.getId().getValue(), Util.fetchIdValueFromElementIds(building1.getVisibleFrom()));
//            }
//
//            try {
//                if (isPrecompute) {
//                    Util.writeObject(observableAreas, observableAreasFileName);
//                    Util.writeObject(visibleFrom, visibleFromFileName);
//                }
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//        }
//    }

    private Collection<TIMRADBuilding> getCSUBuildings() {
        return TIMRADBuildingMap.values();
    }

    private Collection<TIMRADRoad> getTIMRADRoads() {
        return TIMRADRoadMap.values();
    }

//    private void loadVisibilityData(String visibleFromFileName, String observableAreasFileName) {
//        try {
//            FileEntityIDMap visibleFrom = (FileEntityIDMap) Util.readObject(visibleFromFileName);
//            FileEntityIDMap observableArea = (FileEntityIDMap) Util.readObject(observableAreasFileName);
//            for (Integer next : visibleFrom.keySet()) {
//                CSURoad road = this.getCsuRoad(new EntityID(next));
//                if (road != null) {
//                    road.setVisibleFrom(Util.fetchEntityIdFromIdValues(visibleFrom.get(next)));
//                } else {
//                    CSUBuilding building = this.getCsuBuilding(new EntityID(next));
//                    building.setVisibleFrom(Util.fetchEntityIdFromIdValues(visibleFrom.get(next)));
//                }
//            }
//            for (Integer next : observableArea.keySet()) {
//                CSURoad road = this.getCsuRoad(new EntityID(next));
//                if (road != null) {
//                    road.setObservableAreas(new ArrayList<>(Util.fetchEntityIdFromIdValues(observableArea.get(next))));
//                } else {
//                    CSUBuilding building = this.getCsuBuilding(new EntityID(next));
//                    building.setObservableAreas(new ArrayList<>(Util.fetchEntityIdFromIdValues(observableArea.get(next))));
//                }
//            }
//        } catch (IOException | ClassNotFoundException e) {
//            e.printStackTrace();
//        }
//    }

    public long getUniqueMapNumber() {
        return uniqueMapNumber;
    }


    public void setSearchTarget(EntityID searchTarget) {
        this.searchTarget = searchTarget;
    }

    public EntityID getSearchTarget() {
        return searchTarget;
    }

    public Building getBuildingInPoint(int x, int y) {
        String xy = x + "," + y;
        return buildingXYMap.get(xy);
    }

    public int getEntityLastUpdateTime(StandardEntity entity) {
        int maxTime = Integer.MIN_VALUE;
        for (Property property : entity.getProperties()) {
            Integer value = getPropertyTime(property);
            if (value > maxTime) {
                maxTime = value;
            }
        }

        return maxTime;
    }

    public Integer getPropertyTime(Property property) {
        Integer integer = propertyTimeMap.get(property);
        if (integer == null) {
            return 0;
        }
        return integer;
    }

    public Set<Building> getBuildingsInRange(EntityID entityID, int distance) {
        Set<Building> result = new HashSet<>();
        for (StandardEntity e : getObjectsInRange(entityID, distance)) {
            if (e instanceof Building) {
                result.add((Building) e);
            }
        }
        return result;
    }
}