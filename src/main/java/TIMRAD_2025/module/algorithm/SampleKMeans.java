package TIMRAD_2025.module.algorithm;

import TIMRAD_2025.module.algorithm.CompositeConvexHull;
import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.StaticClustering;
import org.apache.commons.math3.ml.clustering.CentroidCluster;
import org.apache.commons.math3.ml.clustering.Clusterable;
import org.apache.commons.math3.ml.clustering.KMeansPlusPlusClusterer;
import org.apache.commons.math3.ml.distance.DistanceMeasure;
import org.apache.commons.math3.random.RandomGenerator;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.*;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;

import java.awt.*;
import java.util.List;
import java.util.*;


public class SampleKMeans extends StaticClustering {

	private int repeatPrecompute;
	private int repeatPreparate;
	private boolean calced = false;
	private boolean assignAgentsFlag;

	private int clusterNumber;
	private int agentSize;
	private int Cluster_AT_number = 1;
	private int Cluster_FB_number = 1;
	private Collection<StandardEntity> entities;
	private ArrayList<ClusterNode> allNodes = new ArrayList<>();
	private ArrayList<StandardEntity> sortedTeamAgents = new ArrayList<>();

	private int entityClusterIdx = -1;
	private int entityIdsClusterIdx = -1;
	private Collection<StandardEntity> lastClusterEntitiesQueryResult = new ArrayList<>();
	private Collection<EntityID> lastClusterEntityIDsQueryResult = new ArrayList<>();
	private List<StandardEntity> centerList;
	private List<EntityID> centerIDs;
	private Map<Integer, List<StandardEntity>> clusterEntitiesList;
	private List<List<EntityID>> clusterEntityIDsList;

	private int allocations[] = null;


	class ClusterNode implements Clusterable {
		public Area area = null;
		public int clusterIndex = 0;
		double point[] = new double[2];

		@Override
		public double[] getPoint() {
			return point;
		}

		public ClusterNode(Area area, int clusterIndex) {
			this.area = area;
			this.clusterIndex = clusterIndex;
			this.point[0] = area.getX();
			this.point[1] = area.getY();
		}

	}

	public SampleKMeans(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
		super(ai, wi, si, moduleManager, developData);
		this.repeatPrecompute = developData.getInteger("sample.module.SampleKMeans.repeatPrecompute", 7);
		this.repeatPreparate = developData.getInteger("sample.module.SampleKMeans.repeatPreparate", 30);
		this.clusterNumber = developData.getInteger("sample.module.SampleKMeans.clusterSize", 10);
		this.assignAgentsFlag = developData.getBoolean("sample.module.SampleKMeans.assignAgentsFlag", true);
		this.clusterEntityIDsList = new ArrayList<>();
		this.centerIDs = new ArrayList<>();
		this.clusterEntitiesList = new HashMap<>();
		this.centerList = new ArrayList<>();
		this.entities = wi.getEntitiesOfType(
				StandardEntityURN.ROAD,
				StandardEntityURN.HYDRANT,
				StandardEntityURN.BUILDING,
				StandardEntityURN.REFUGE,
				StandardEntityURN.GAS_STATION,
				StandardEntityURN.AMBULANCE_CENTRE,
				StandardEntityURN.FIRE_STATION,
				StandardEntityURN.POLICE_OFFICE
		);
		if (agentInfo.me().getStandardURN().equals(StandardEntityURN.POLICE_FORCE))//如果是警察
		{
			this.entities = wi.getEntitiesOfType(
					StandardEntityURN.ROAD,
					StandardEntityURN.HYDRANT,
					StandardEntityURN.BUILDING,
					StandardEntityURN.REFUGE,
					StandardEntityURN.GAS_STATION,
					StandardEntityURN.AMBULANCE_CENTRE,
					StandardEntityURN.FIRE_STATION,
					StandardEntityURN.POLICE_OFFICE
			);
		}
		else
		{
			this.entities = wi.getEntitiesOfType(
					StandardEntityURN.BUILDING,
					StandardEntityURN.REFUGE,
					StandardEntityURN.AMBULANCE_CENTRE,
					StandardEntityURN.FIRE_STATION,
					StandardEntityURN.POLICE_OFFICE
			);
		}
		if (agentInfo.me().getStandardURN().equals(StandardEntityURN.AMBULANCE_TEAM)) {
			agentSize = scenarioInfo.getScenarioAgentsAt();
		} else if (agentInfo.me().getStandardURN().equals(StandardEntityURN.FIRE_BRIGADE)) {
			agentSize = scenarioInfo.getScenarioAgentsFb();
		} else if (agentInfo.me().getStandardURN().equals(StandardEntityURN.POLICE_FORCE)) {
			agentSize = scenarioInfo.getScenarioAgentsPf();
		}

		clusterNumber = Math.min(30, agentSize);
	}

	@Override
	public Clustering updateInfo(MessageManager messageManager) {
		super.updateInfo(messageManager);
		if (this.getCountUpdateInfo() >= 2) {
			return this;
		}
		this.centerList.clear();
		this.clusterEntitiesList.clear();
		return this;
	}

	@Override
	public Clustering precompute(PrecomputeData precomputeData) {
		super.precompute(precomputeData);

		if (this.calced) {
			return this;
		}

		calClusterAndAssign();
		return this;
	}

	@Override
	public Clustering resume(PrecomputeData precomputeData) {
		super.resume(precomputeData);
		if (this.calced) {
			return this;
		}
		calClusterAndAssign();
		// visualDebug();
		return this;
	}

	@Override
	public Clustering preparate() {
		super.preparate();

		if (this.calced) {
			return this;
		}

		calClusterAndAssign();
		// visualDebug();
		return this;
	}


	private void assignAgent() {
		allocations = new int[sortedTeamAgents.size()];
		double clusterCenters[][] = new double[clusterNumber][3];
		for (int i = 0; i < clusterNumber; i++) {
			clusterCenters[i][0] = 0;
			clusterCenters[i][1] = 0;
			clusterCenters[i][2] = 0;
		}
		for (ClusterNode clusterNode : allNodes) {
			clusterCenters[clusterNode.clusterIndex][0] += clusterNode.point[0];
			clusterCenters[clusterNode.clusterIndex][1] += clusterNode.point[1];
			clusterCenters[clusterNode.clusterIndex][2] += 1;
		}
		for (int i = 0; i < clusterNumber; i++) {
			if (clusterCenters[i][2] != 0) {
				clusterCenters[i][0] /= clusterCenters[i][2];
				clusterCenters[i][1] /= clusterCenters[i][2];
			}
		}

		double costs[][] = new double[sortedTeamAgents.size()][clusterNumber];
		for (int i = 0; i < costs.length; i++) {
			for (int j = 0; j < costs[0].length; j++) {
				StandardEntity agentStd = sortedTeamAgents.get(i);
				double ax = (int) (agentStd.getProperty(4614).getValue());
				double ay = (int) (agentStd.getProperty(4615).getValue());
//				double ax = (int) (agentStd.getProperty("urn:rescuecore2.standard:property:x").getValue());
//				double ay = (int) (agentStd.getProperty("urn:rescuecore2.standard:property:y").getValue());
//				double ax = (int) (agentStd.getProperty(1).getValue());
//				double ay = (int) (agentStd.getProperty(1).getValue());

				double dist = dist(ax, ay, clusterCenters[j][0], clusterCenters[j][1]);
				costs[i][j] = dist;
			}
		}
		HungarianAgentAssign hungarianAgentAssign = new HungarianAgentAssign(costs);

		allocations = hungarianAgentAssign.execute();
	}

	private void calClusterAndAssign(){

		allNodes.clear();
		sortedTeamAgents.clear();
		sortedTeamAgents.addAll(worldInfo.getEntitiesOfType(agentInfo.me().getStandardURN()));
		Collections.sort(sortedTeamAgents, new Comparator<StandardEntity>(){
			@Override
			public int compare(StandardEntity o1, StandardEntity o2) {
				return o1.getID().getValue() - o2.getID().getValue();
			}
		});
		this.clusterNumber = Math.max(1, sortedTeamAgents.size());
		if(agentInfo.me().getStandardURN().equals(StandardEntityURN.AMBULANCE_TEAM)){
			this.clusterNumber=(this.clusterNumber+Cluster_AT_number-1)/Cluster_AT_number;
		}
		if(agentInfo.me().getStandardURN().equals(StandardEntityURN.FIRE_BRIGADE)){
			this.clusterNumber=(this.clusterNumber+Cluster_FB_number-1)/Cluster_FB_number;
		}
		Collection<StandardEntity> allStandardEntity =  this.worldInfo.getEntitiesOfType(//所有建筑
				StandardEntityURN.FIRE_STATION,
				StandardEntityURN.POLICE_OFFICE,
				StandardEntityURN.AMBULANCE_CENTRE,
				StandardEntityURN.REFUGE,
				StandardEntityURN.GAS_STATION,
				StandardEntityURN.BUILDING,
				StandardEntityURN.ROAD,
				StandardEntityURN.HYDRANT
		);

		for (StandardEntity standardEntity : allStandardEntity) {
			allNodes.add(new ClusterNode((Area) standardEntity, 0));
		}

		DistanceMeasure distanceMeasure = new DistanceMeasure() {
			@Override
			public double compute(double[] doubles, double[] doubles1) {
				return 0;
			}
		};
		RandomGenerator randomGenerator =  new RandomGenerator() {
			@Override
			public void setSeed(int i) {

			}

			@Override
			public void setSeed(int[] ints) {

			}

			@Override
			public void setSeed(long l) {

			}

			@Override
			public void nextBytes(byte[] bytes) {

			}

			@Override
			public int nextInt() {
				return 0;
			}

			@Override
			public int nextInt(int i) {
				return 0;
			}

			@Override
			public long nextLong() {
				return 0;
			}

			@Override
			public boolean nextBoolean() {
				return false;
			}

			@Override
			public float nextFloat() {
				return 0;
			}

			@Override
			public double nextDouble() {
				return 0;
			}

			@Override
			public double nextGaussian() {
				return 0;
			}
		};
		KMeansPlusPlusClusterer<ClusterNode> dbscan = new KMeansPlusPlusClusterer<ClusterNode>(clusterNumber, 30);
		dbscan.getRandomGenerator().setSeed(agentInfo.me().getStandardURN().ordinal() + 1);

		List<CentroidCluster<ClusterNode>> dbscanCluster = dbscan.cluster(allNodes);
		int clusterIndex = 0;

		for (CentroidCluster<ClusterNode> centroidCluster : dbscanCluster) {
			for (ClusterNode clusterNode : centroidCluster.getPoints()) {
				clusterNode.clusterIndex = clusterIndex;
			}
			clusterIndex++;
		}
		if(!agentInfo.me().getStandardURN().equals(StandardEntityURN.POLICE_FORCE)){
			assignAT_FB();
		}
		else
			assignAgent();
		this.calced = true;
		// visualDebug();
	}
	private void assignAT_FB() {
		ArrayList<StandardEntity> agentList = new ArrayList<>(sortedTeamAgents);
		allocations = new int[sortedTeamAgents.size()];
		double clusterCenters[][] = new double[clusterNumber][3];
		for (int i = 0; i < clusterNumber; i++) {
			clusterCenters[i][0] = 0;
			clusterCenters[i][1] = 0;
			clusterCenters[i][2] = 0;
		}
		for (ClusterNode clusterNode : allNodes) {
			clusterCenters[clusterNode.clusterIndex][0] += clusterNode.point[0];
			clusterCenters[clusterNode.clusterIndex][1] += clusterNode.point[1];
			clusterCenters[clusterNode.clusterIndex][2] += 1;
		}
		for (int i = 0; i < clusterNumber; i++) {
			if (clusterCenters[i][2] != 0) {
				clusterCenters[i][0] /= clusterCenters[i][2];
				clusterCenters[i][1] /= clusterCenters[i][2];
			}
		}
		double costs[][] = new double[sortedTeamAgents.size()][clusterNumber];
		for (int i = 0; i < costs.length; i++) {
			for (int j = 0; j < costs[0].length; j++) {
				StandardEntity agentStd = sortedTeamAgents.get(i);
//				double ax = (int) (agentStd.getProperty("urn:rescuecore2.standard:property:x").getValue());
//				double ay = (int) (agentStd.getProperty("urn:rescuecore2.standard:property:y").getValue());
				double ax = (int) (agentStd.getProperty(4614).getValue());
				double ay = (int) (agentStd.getProperty(4615).getValue());

//				double ax = (int) (agentStd.getProperty(1).getValue());
//				double ay = (int) (agentStd.getProperty(1).getValue());
				double dist = dist(ax, ay, clusterCenters[j][0], clusterCenters[j][1]);
				costs[i][j] = dist;
			}
		}
		int clusterIndex = 0;
		while (agentList.size() > 0) {
			StandardEntity agent = this.getNearestAgent(costs, agentList, clusterIndex);
			int agent_number=sortedTeamAgents.indexOf(agent);
			allocations[agent_number]=clusterIndex;

			agentList.remove(agent);
			clusterIndex++;
			if (clusterIndex >= this.clusterNumber) {
				clusterIndex = 0;
			}
		}
	}
	private StandardEntity getNearestAgent(double[][] costMatrix, ArrayList<StandardEntity> srcAgentList, int ClusterIdx) {
		StandardEntity result = null;
		double cost = Integer.MAX_VALUE;
		for (StandardEntity agent : srcAgentList) {
			if (result == null) {
				result = agent;
			} else {
					if (costMatrix[sortedTeamAgents.indexOf(agent)][ClusterIdx] < cost) {
						result = agent;
						cost = costMatrix[sortedTeamAgents.indexOf(agent)][ClusterIdx];
					}
			}
		}
		return result;
	}
	@Override
	public int getClusterNumber() {
		return clusterNumber;
	}

	@Override
	public int getClusterIndex(StandardEntity entity) {
		if (sortedTeamAgents.contains(entity)) {
			return getAgentInitialClusterIndex(entity);
		}
		return getClusterIndex(entity.getID());
	}

	@Override
	public int getClusterIndex(EntityID id) {
		StandardEntity entity = this.worldInfo.getEntity(id);
		if (entity != null && sortedTeamAgents.contains(entity)) {
			return getAgentInitialClusterIndex(entity);
		}
		for (ClusterNode clusterNode : allNodes) {
			if (clusterNode.area.getID().equals(id)) {
				return clusterNode.clusterIndex;
			}
		}
		return -1;
	}

	private int getAgentInitialClusterIndex(StandardEntity agent) {
		if (allocations == null) {
			return -1;
		}
		return allocations[sortedTeamAgents.indexOf(agent)];
	}

	@Override
	public Collection<EntityID> getClusterEntityIDs(int index) {
		Collection<EntityID> result = new ArrayList<>();
		if (entityIdsClusterIdx != index) {
			lastClusterEntityIDsQueryResult.clear();
			for (ClusterNode clusterNode : allNodes) {
				if (clusterNode.clusterIndex == index) {
					lastClusterEntityIDsQueryResult.add(clusterNode.area.getID());
				}
			}
		}
		entityIdsClusterIdx = index;
		result.addAll(lastClusterEntityIDsQueryResult);
		return result;
	}
	@Override
	public Collection<StandardEntity> getClusterEntities(int index) {
		Collection<StandardEntity> result = new ArrayList<>();
		if (entityClusterIdx != index) {
			lastClusterEntitiesQueryResult.clear();
			for (ClusterNode clusterNode : allNodes) {
				if (clusterNode.clusterIndex == index) {
					lastClusterEntitiesQueryResult.add(clusterNode.area);
				}
			}
		}
		entityClusterIdx = index;
		result.addAll(lastClusterEntitiesQueryResult);
		return result;
	}

	@Override
	public Clustering calc() {
		return this;
	}



	// private void visualDebug() {
	// 	int index = getClusterIndex(agentInfo.getID());
	// 	CompositeConvexHull convexHull = new CompositeConvexHull();
	// 	Collection<StandardEntity> clusterEntities = getClusterEntities(index);
	// 	clusterEntities.remove(agentInfo.me());
	// 	for (StandardEntity entity : getClusterEntities(index)) {
	// 		Pair<Integer, Integer> location = worldInfo.getLocation(entity);
	// 		convexHull.addPoint(location.first(), location.second());
	// 	}
	// 	Polygon polygon = convexHull.getConvexPolygon();
	// 	ArrayList<Polygon> data = new ArrayList<>();
	// 	if (polygon != null) {
	// 		data.add(convexHull.getConvexPolygon());
	// 	}
//		if (DebugHelper.DEBUG_MODE && scenarioInfo.getMode() != ScenarioInfo.Mode.PRECOMPUTATION_PHASE) {
//			try {
//				DebugHelper.VD_CLIENT.drawAsync(agentInfo.getID().getValue(), "ClusterConvexPolygon", data);
//				if (agentInfo.me() instanceof FireBrigade) {
//					DebugHelper.VD_CLIENT.drawAsync(agentInfo.getID().getValue(), "FBClusterConvexPolygon", data);
//				}
//				if (agentInfo.me() instanceof PoliceForce) {
//					DebugHelper.VD_CLIENT.drawAsync(agentInfo.getID().getValue(), "PFClusterConvexPolygon", data);
//				}
//				if (agentInfo.me() instanceof AmbulanceTeam) {
//					DebugHelper.VD_CLIENT.drawAsync(agentInfo.getID().getValue(), "ATClusterConvexPolygon", data);
//				}
//			} catch (Exception e) {
//				e.printStackTrace();
//			}
//		}
	// }

	public static double dist(double Ax, double Ay, double Bx, double By) {
		return Math.hypot(Ax - Bx, Ay - By);
	}

}
