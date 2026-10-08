# Research Notes

## Problem

An autonomous AGV must move from a start pose to a goal pose through a structured industrial environment containing static obstacles.

## Baseline

The baseline is an 8-connected A* planner operating on a binary occupancy grid.

## Important limitation

A* plans for a point robot. It does not yet account for vehicle footprint, orientation, wheel constraints, or tracking dynamics.

That limitation motivates M2.

## Experimental principle

Every new method should be compared against the same:
- map
- start and goal
- obstacle configuration
- resolution
- evaluation metrics

This makes the later comparison scientifically meaningful.
