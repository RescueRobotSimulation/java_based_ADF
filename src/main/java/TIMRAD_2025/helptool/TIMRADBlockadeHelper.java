package TIMRAD_2025.helptool;

import adf.core.agent.info.WorldInfo;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.worldmodel.EntityID;

import java.awt.*;
import java.util.LinkedList;
import java.util.List;

public class TIMRADBlockadeHelper {

	private Polygon polygon;

	private Blockade underlyingBlockade;

	private EntityID blockadeId;

	private List<Pair<Integer, Integer>> vertexes = new LinkedList<>();

	public TIMRADBlockadeHelper(EntityID blockadeId, WorldInfo world) {
		this.underlyingBlockade = getEntity(world, blockadeId, Blockade.class);
		this.blockadeId = blockadeId;

		this.polygon = createPolygon(underlyingBlockade.getApexes());
	}

	public TIMRADBlockadeHelper(EntityID id, int[] apexes, int x, int y) {
		this.underlyingBlockade = new Blockade(blockadeId);

		this.underlyingBlockade.setX(x);
		this.underlyingBlockade.setY(y);
		this.underlyingBlockade.setApexes(apexes);

		this.blockadeId = id;
		this.polygon = createPolygon(apexes);
	}

	private Polygon createPolygon(int[] apexes) {
		int vertexCount = apexes.length / 2;
		int[] xCoordinates = new int[vertexCount];
		int[] yCOordinates = new int[vertexCount];

		for (int i = 0; i < vertexCount; i++) {
			xCoordinates[i] = apexes[2 * i];
			yCOordinates[i] = apexes[2 * i + 1];

			vertexes.add(new Pair<Integer, Integer>(apexes[2 * i], apexes[2 * i + 1]));
		}

		return new Polygon(xCoordinates, yCOordinates, vertexCount);
	}

	public Polygon getPolygon() {
		return this.polygon;
	}

	public Blockade getSelfBlockade() {
		return this.underlyingBlockade;
	}

	public EntityID getBlockadeId() {
		return this.blockadeId;
	}

	public List<Pair<Integer, Integer>> getVertexesList() {
		return this.vertexes;
	}

	public <T extends StandardEntity> T getEntity(WorldInfo world, EntityID id, Class<T> c) {
		StandardEntity entity;
		entity = world.getEntity(id);
		if (c.isInstance(entity)) {
			T castedEntity = c.cast(entity);

			return castedEntity;
		} else {
			return null;
		}
	}
}
