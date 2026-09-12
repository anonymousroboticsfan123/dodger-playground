

export function getActiveDpcbfConstraintIds(diagnostics, enabled = true) {
    if (!enabled || diagnostics?.enabled === false) return new Set();

    if (Array.isArray(diagnostics?.constrainedObstacleIds)) {
        return new Set(diagnostics.constrainedObstacleIds.map(String));
    }

    if (Array.isArray(diagnostics?.overlays)) {
        return new Set(
            diagnostics.overlays
                .map((overlay) => overlay?.obstacleId)
                .filter((id) => id != null)
                .map(String)
        );
    }

    return new Set(
        Array.isArray(diagnostics?.activeObstacleIds)
            ? diagnostics.activeObstacleIds.map(String)
            : []
    );
}
