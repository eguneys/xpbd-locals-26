export type Vec2 = { x: number, y: number }

export type Poly = Vec2[]

function distPointToSegment(p: Vec2, a: Vec2, b: Vec2) {
    const ab = {x: b.x - a.x, y: b.y - a.y};
    const ap = {x: p.x - a.x, y: p.y - a.y};
    const abLen2 = ab.x * ab.x + ab.y * ab.y;
    const t = Math.max(0, Math.min(1, (ap.x * ab.x + ap.y * ab.y) / abLen2));
    const closest = {x: a.x + ab.x * t, y: a.y + ab.y * t};
    const dx = p.x - closest.x, dy = p.y - closest.y;
    return Math.sqrt(dx*dx + dy*dy);
}

export function clickedEdge(point: Vec2, poly: Poly, threshold = 8): number | undefined {
    for (let i = 0; i < poly.length; i++) {
        const j = (i + 1) % poly.length;
        if (distPointToSegment(point, poly[i], poly[j]) <= threshold) {
            return i; // edge starting at i
        }
    }
    return undefined;
}