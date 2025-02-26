package TIMRAD_2025.helptool.object;

import rescuecore2.misc.geometry.Line2D;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class TIMRADEscapePoint {
	private List<TIMRADBlockade> realteBlockades = new ArrayList<>();;
	
	private Point underlyingPoint;
	
	private Line2D line;
	
	public TIMRADEscapePoint(Point point, Line2D line, TIMRADBlockade... blockade) {
		this.setUnderlyingPoint(point);
		this.setLine(line);
		
		for (TIMRADBlockade next : blockade) {
			this.realteBlockades.add(next);
		}
	}

	public List<TIMRADBlockade> getRelateBlockade() {
		return this.realteBlockades;
	}
	
	public void addCsuBlockade(TIMRADBlockade blockade) {
		this.realteBlockades.add(blockade);
	}
	
	public boolean removeCsuBLockade(TIMRADBlockade blockade) {
		return this.realteBlockades.remove(blockade);
	}

	public Point getUnderlyingPoint() {
		return underlyingPoint;
	}

	public void setUnderlyingPoint(Point underlyingPoint) {
		this.underlyingPoint = underlyingPoint;
	}

	public Line2D getLine() {
		return line;
	}

	public void setLine(Line2D line) {
		this.line = line;
	}
}
