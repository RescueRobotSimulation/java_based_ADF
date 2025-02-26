package TIMRAD_2025.helptool;

import TIMRAD_2025.module.algorithm.CompositeConvexHull;

//import javolution.util.FastSet;
import javolution.util.FastSet;
//import javolution.util.FastSet;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Refuge;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.misc.geometry.Point2D;


import java.awt.*;
import java.util.Set;

/**
 * Tools for scaling polygon.
 * 
 * @author TIMRAD
 */
public class PolygonScaler {

	public static Polygon scalePolygon(Polygon sourcePolygon, double scale) {
		Polygon scaledPolygon;
		Point2D p1, p2;
		int[] xs = new int[sourcePolygon.npoints];
		int[] ys = new int[sourcePolygon.npoints];

		// for (int i = 0; i < sourcePolygon.npoints; i++) {
		// 	p1 = new Point2D(sourcePolygon.xpoints[i], sourcePolygon.ypoints[i]);
		// 	p2 = p1.scale(scale);
		// 	xs[i] = (int) p2.getX();
		// 	ys[i] = (int) p2.getY();
		// 	p1.clone();
		// }

		Polygon preScaledPolygon = new Polygon(xs, ys, sourcePolygon.npoints);
		scaledPolygon = reAllocatePolygon(preScaledPolygon, sourcePolygon);///place at the center position
		if (scaledPolygon == null)
			scaledPolygon = preScaledPolygon;

		return scaledPolygon;
	}

	private static Polygon reAllocatePolygon(Polygon scaled, Polygon source) {
		if (source == null || scaled == null || source.npoints == 0 || scaled.npoints == 0)
			return null;

		Polygon reAllocated;
		int[] xs = new int[scaled.npoints];
		int[] ys = new int[scaled.npoints];

		int sourceCenterX = 0;
		int sourceCenterY = 0;

		int scaledCenterX = 0;
		int scaledCenterY = 0;

		for (int i = 0; i < scaled.npoints; i++) {
			sourceCenterX += source.xpoints[i];
			sourceCenterY += source.ypoints[i];

			scaledCenterX += scaled.xpoints[i];
			scaledCenterY += scaled.ypoints[i];
		}

		sourceCenterX /= source.npoints;
		sourceCenterY /= source.npoints;

		scaledCenterX /= scaled.npoints;
		scaledCenterY /= scaled.npoints;

		int xDistance = sourceCenterX - scaledCenterX;
		int yDistance = sourceCenterY - scaledCenterY;
        ///then the center of the scaled will be at the same position with the source
		for (int i = 0; i < scaled.npoints; i++) {
			xs[i] = scaled.xpoints[i] + xDistance;
			ys[i] = scaled.ypoints[i] + yDistance;
		} 

		reAllocated = new Polygon(xs, ys, scaled.npoints);
		return reAllocated;
	}

	public static Set<StandardEntity> getMapBorderBuildings(CompositeConvexHull convexHull,
															Set<StandardEntity> entities, double scale) {
		Building building;
		Polygon convexHullPolygon = convexHull.getConvexPolygon();
		Set<StandardEntity> borderEntities = new FastSet<StandardEntity>();
       ///when will occur ?
		if (convexHullPolygon.npoints == 0) {
			System.out.println("Something gone wrong in setting border entities for Firebrigade!!!");
			return null;
		}

		Polygon smallBorderPolygon = scalePolygon(convexHullPolygon, scale);
		Polygon bigBorderPolygon = scalePolygon(convexHullPolygon, 2 - scale);

		for (StandardEntity entity : entities) {
			if (entity instanceof Refuge)
				continue;
			if (!(entity instanceof Building))
				continue;
			building = (Building) entity;
			int[] vertices = building.getApexList();
			for (int i = 0; i < vertices.length; i += 2) {
				if ((bigBorderPolygon.contains(vertices[i], vertices[i + 1]))
						&& !(smallBorderPolygon.contains(vertices[i], vertices[i + 1]))) {
					borderEntities.add(entity);
					break;
				}
			}
		}
		return borderEntities;
	}
}
