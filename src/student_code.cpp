#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    std::vector<Vector2D> new_points(points.size() - 1);
    
    for (int i = 0; i < new_points.size(); i++) {
      new_points[i] = (1 - t) * points[i] + t * points[i + 1];
    }
    return new_points;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> new_points(points.size() - 1);

    for (int i = 0; i < new_points.size(); i++) {
      new_points[i] = (1 - t) * points[i] + t * points[i + 1];
    }
    return new_points;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    std::vector<Vector3D> new_points(points);
    while (new_points.size() > 1) {
      new_points = evaluateStep(new_points, t);
    }
    return new_points[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    std::vector<Vector3D> new_points(controlPoints.size());
    for (int i = 0; i < controlPoints.size(); i++) {
      new_points[i] = evaluate1D(controlPoints[i], u);
    }
    return evaluate1D(new_points, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    HalfedgeCIter g = halfedge();
    HalfedgeCIter h(g);

    Vector3D normal(0);
    do {
      Vector3D h1 = h->vertex()->position;
      h1 -= h->twin()->vertex()->position;
      h = h->twin()->next();
      Vector3D h2 = h->vertex()->position;
      h2 -= h->twin()->vertex()->position;

      normal += cross(h2, h1);
    } while (h != g);
    normal.normalize();
    return normal;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    
    if (e0->isBoundary()) {
      return e0;
    }

    // For each vertex, edge, and face, set its halfedge pointer.
    // set new half-edge for halfedge()->vertex()
    HalfedgeIter h = e0->halfedge();
    VertexIter v = h->vertex();
    if (v->halfedge() == h) {
      v->halfedge() = h->twin()->next();
    }
    // set new half-edge for halfedge()->twin()->vertex()
    v = h->twin()->vertex();
    if (v->halfedge() == h) {
      v->halfedge() = h->twin()->next();
    }
    // For each half-edge, set its next, twin, vertex, edge, and face pointer to the correct element.You can use Halfedge::setNeighbors(...) to set all pointers of a half - edge at once.
    HalfedgeIter prev = h->next()->next();
    HalfedgeIter twinPrev = h->twin()->next()->next();
    prev->next() = h->twin()->next();
    twinPrev->next() = h->next();
    h->next()->next() = h;
    h->twin()->next()->next() = h->twin();
    prev->face() = h->twin()->face();
    twinPrev->face() = h->face();
    h->setNeighbors(twinPrev,
      h->twin(),
      prev->vertex(),
      e0,
      h->face());
    h->twin()->setNeighbors(prev,
      h,
      twinPrev->vertex(),
      e0,
      h->twin()->face());

    h->face()->halfedge() = h;
    h->twin()->face()->halfedge() = h->twin();
    
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary()) {
      return e0->halfedge()->vertex();
    }

    std::vector<HalfedgeIter> hh = { e0->halfedge(), e0->halfedge()->twin() };
    Vector3D p = 0.5 * (hh[0]->vertex()->position + hh[1]->vertex()->position);

    VertexIter vertex = newVertex();
    vertex->position = p;
    vertex->isNew = true;
    std::vector<EdgeIter> newEdges(2);
    std::vector<EdgeIter> oldEdgeSegments = { e0, newEdge() };
    oldEdgeSegments[1]->isNew = false;
    std::vector<FaceIter> newFaces(2);

    std::vector<HalfedgeIter> mainHalfedges(2);
    std::vector<HalfedgeIter> twinHalfedges(2);
    std::vector<HalfedgeIter> splitHalfedges(2);
    // starter code doesn't manage boundaries well - no good way to do this
    bool boundary = e0->isBoundary();
    for (int i = 0; i < 2 - boundary; i++) {
      mainHalfedges[i] = newHalfedge();
      twinHalfedges[i] = newHalfedge();
      splitHalfedges[i] = newHalfedge();
      newEdges[i] = newEdge();
      newFaces[i] = boundary ? newBoundary() : newFace();
      newEdges[i]->isNew = true;
    }
    for (int i = 0; i < 2 - boundary; i++) {
      mainHalfedges[i]->setNeighbors(
        hh[i]->next()->next(),
        twinHalfedges[i],
        vertex,
        newEdges[i],
        hh[i]->face()
      );

      twinHalfedges[i]->setNeighbors(
        splitHalfedges[i],
        mainHalfedges[i],
        hh[i]->next()->next()->vertex(),
        newEdges[i],
        newFaces[i]
      );

      splitHalfedges[i]->setNeighbors(
        hh[i]->next(),
        hh[1 - i],
        vertex,
        oldEdgeSegments[1 - i],
        newFaces[i]
      );

      // set outer edges
      hh[i]->next()->next() = twinHalfedges[i];
      hh[i]->next()->face() = newFaces[i];
      hh[i]->next() = mainHalfedges[i];

      // set faces
      hh[i]->face()->halfedge() = hh[i];

      newEdges[i]->halfedge() = mainHalfedges[i];
      newFaces[i]->halfedge() = splitHalfedges[i];
      hh[i]->twin() = splitHalfedges[1 - i];
    }

    oldEdgeSegments[1]->halfedge() = splitHalfedges[0];
    vertex->halfedge() = splitHalfedges[0];

    return vertex;
  }

  void Vertex::computeCentroid() {
    HalfedgeCIter h = halfedge();
    HalfedgeCIter g(h);
    Vector3D cent(0);
    int deg = 0;
    do {
      g = g->twin();
      cent += g->vertex()->position;
      g = g->next();
      deg++;
    } while (h != g);
    centroid = cent;
  }

  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.

    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      int n = v->degree();
      float u = n == 3 ? 3.0f / 16.0f : 3.0f / (8.0f * n);
      v->computeCentroid();
      v->newPosition = (1 - n * u)* v->position + u * v->centroid;
      v->isNew = false;
    }
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
 
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      Vector3D a = e->halfedge()->vertex()->position;
      Vector3D b = e->halfedge()->twin()->vertex()->position;
      Vector3D c = e->halfedge()->next()->next()->vertex()->position;
      Vector3D d = e->halfedge()->twin()->next()->next()->vertex()->position;
      e->newPosition = 0.375 * (a + b) + 0.125 * (c + d);
    }

    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

    EdgeIter e = mesh.edgesBegin();
    int nEdges = mesh.nEdges();
    // Can't use mesh.edgesEnd() because list is being appended to - end remains valid but moves with the list.
    for (int i = 0; i < nEdges; i++, e++) {
      mesh.splitEdge(e);
    }

    // 4. Flip any new edge that connects an old and new vertex.

    for (; e != mesh.edgesEnd(); e++) {
      // Flip any completely new edge that connects a new vertex to an old one.
      if (e->isNew && e->halfedge()->vertex()->isNew != e->halfedge()->twin()->vertex()->isNew) {
        mesh.flipEdge(e);
      }
    }

    // 5. Copy the new vertex positions into final Vertex::position.

    e = mesh.edgesBegin();
    for (int i = 0; i < nEdges; i++, e++) {
      e->halfedge()->twin()->vertex()->newPosition = e->newPosition;
    }

    std::for_each(mesh.verticesBegin(), mesh.verticesEnd(), [](Vertex& v) { v.position = v.newPosition; });
    
  }
}
