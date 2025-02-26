package TIMRAD_2025.helptool;

import rescuecore2.misc.geometry.Line2D;

import java.awt.*;
import java.util.LinkedList;
import java.util.List;

public class TIMRADEscapePointHelper {
	private List<TIMRADBlockadeHelper> realteBlockades = new LinkedList<>();;
	
	private Point underlyingPoint;
	
	private Line2D line;
	
	public TIMRADEscapePointHelper(Point point, Line2D line, TIMRADBlockadeHelper... blockade) {
		this.setUnderlyingPoint(point);
		this.setLine(line);
		
		for (TIMRADBlockadeHelper next : blockade) {
			this.realteBlockades.add(next);
		}
	}

	public List<TIMRADBlockadeHelper> getRelateBlockade() {
		return this.realteBlockades;
	}
	
	public void addTIMRADBlockade(TIMRADBlockadeHelper blockade) {
		this.realteBlockades.add(blockade);
	}
	
	public boolean removeTIMRADBLockade(TIMRADBlockadeHelper blockade) {
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
