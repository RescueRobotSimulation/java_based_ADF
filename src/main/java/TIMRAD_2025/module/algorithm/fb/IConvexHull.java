package TIMRAD_2025.module.algorithm.fb;

import java.awt.*;
import java.util.Collection;

public interface IConvexHull {

    public Long getGuid();

    public Polygon getConvexPolygon();

    public void addPoint(int x, int y);

    public void addPoint(Point point);

    public void removePoint(int x, int y);

    public void removePoint(Point point);

    public void updatePoints(Collection<Point> addedPoints, Collection<Point> removedPoints);
}
