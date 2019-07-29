defmodule ElixirRigidPhysics.Geometry.LineSegment do

  @moduledoc """
  Line segment geometry module.

  Line segemnts go from a start point `a` to an end point `b`.
  """
  require Record
  Record.defrecord(:line_segment, a: {0.0, 0.0, 0.0}, b: {0.0, 0.0, 0.0})
  @type line_segment :: record(:line_segment, a: Vec3.vec3, b: Vec3.vec3)
  alias Graphmath.Vec3

  @doc """
  Creates a line segment.

  ## Examples
    iex> # IO.puts("Test line segment creation.")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> LineSegment.create( {0.0, 1.0, 0.0}, {0.0, 1.0, 1.0} )
    {:line_segment, {0.0, 1.0, 0.0}, {0.0, 1.0, 1.0} }
  """
  @spec create(Vec3.vec3, Vec3.vec3) :: line_segment
  def create(a, b), do: line_segment(a: a, b: b)


  @doc """
  Projects a query point `q` onto the same line as line segment `ab`. **Note that the point may not be on the segment itself.**

  ## Examples
    iex> #IO.puts("Check segment coincident with a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {0.0, 0.0, 0.0}
    iex> LineSegment.project( segment, q)
    {0.0, 0.0, 0.0}

    iex> #IO.puts("Check segment coincident with b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {1.0, 0.0, 0.0}
    iex> LineSegment.project( segment, q)
    {1.0, 0.0, 0.0}

    iex> #IO.puts("Check segment interior of segment")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {0.5, 0.0, 0.0}
    iex> LineSegment.project( segment, q)
    {0.5, 0.0, 0.0}

    iex> #IO.puts("Check segment in voronoi region of a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {-0.5, 0.0, 0.0}
    iex> LineSegment.project( segment, q)
    {-0.5, 0.0, 0.0}

    iex> #IO.puts("Check segment in voronoi region of b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {1.5, 0.0, 0.0}
    iex> LineSegment.project( segment, q)
    {1.5, 0.0, 0.0}

    iex> #IO.puts("Check offset segment in voronoi region of a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {-0.5, 3.0, 4.0}
    iex> LineSegment.project( segment, q)
    {-0.5, 0.0, 0.0}

    iex> #IO.puts("Check offset segment in voronoi region of b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {1.5, 3.0, 4.0}
    iex> LineSegment.project( segment, q)
    {1.5, 0.0, 0.0}

    iex> #IO.puts("Check offset segment in voronoi region of segment")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {0.45, 3.0, 4.0}
    iex> LineSegment.project( segment, q)
    {0.45, 0.0, 0.0}
  """
  @spec project( line_segment, Vec3.vec3) :: Vec3.vec3
  def project( line_segment(a: a, b: b),  q) do
    dir = Vec3.subtract(b, a) |> Vec3.normalize()
    Vec3.add(a, Vec3.scale( dir, Vec3.dot(q, dir)))
  end

  @doc """
  Gets the [Barycentric coordinates](https://en.wikipedia.org/wiki/Barycentric_coordinate_system) of a query point projected on the segment.

  See the [2010 Catto Erin GJK presentation](https://code.google.com/archive/p/box2d/downloads) for details.

  ## Examples
    iex> #IO.puts("Check segment coincident with a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {0.0, 0.0, 0.0}
    iex> LineSegment.to_barycentric( segment, q)
    {1.0, 0.0}

    iex> #IO.puts("Check segment coincident with b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {1.0, 0.0, 0.0}
    iex> LineSegment.to_barycentric( segment, q)
    {0.0, 1.0}

    iex> #IO.puts("Check segment interior of segment")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {0.5, 0.0, 0.0}
    iex> LineSegment.to_barycentric( segment, q)
    {0.5, 0.5}

    iex> #IO.puts("Check segment in voronoi region of a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {-0.5, 0.0, 0.0}
    iex> LineSegment.to_barycentric( segment, q)
    {1.5, -0.5}

    iex> #IO.puts("Check segment in voronoi region of b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {1.5, 0.0, 0.0}
    iex> LineSegment.to_barycentric( segment, q)
    {-0.5, 1.5}

    iex> #IO.puts("Check offset segment in voronoi region of a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {-0.5, 3.0, 4.0}
    iex> LineSegment.to_barycentric( segment, q)
    {1.5, -0.5}

    iex> #IO.puts("Check offset segment in voronoi region of b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {1.5, 3.0, 4.0}
    iex> LineSegment.to_barycentric( segment, q)
    {-0.5, 1.5}

    iex> #IO.puts("Check offset segment in voronoi region of segment")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {0.5, 3.0, 4.0}
    iex> LineSegment.to_barycentric( segment, q)
    {0.5, 0.5}
  """
  @spec to_barycentric( line_segment, Vec3.vec3) :: {float, float}
  def to_barycentric( line_segment(a: a, b: b), q) do

    segment_vec = Vec3.subtract(b,a)
    segment_vec_length = Vec3.length(segment_vec)
    segment_vec_unit = Vec3.scale(segment_vec, 1.0 / segment_vec_length)
    qa = Vec3.subtract(q,a)
    bq = Vec3.subtract(b,q)

    u = Vec3.dot(bq, segment_vec_unit) / segment_vec_length
    v = Vec3.dot(qa, segment_vec_unit) / segment_vec_length

    {u,v}
  end

  @doc """
  Gets a point collinear (and possible coincident) with line segment given barycentric coordinates.

  Note that the sum of the coordinates must equal 1 for this to return a useful value.

  ## Eaxmples
    iex> #IO.puts("Check coordinate at start of line")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
    iex> q = {1.0, 0.0}
    iex> LineSegment.from_barycentric( segment, q)
    {-1.0, -1.0, -1.0}

    iex> #IO.puts("Check coordinate at end of line")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
    iex> q = {0.0, 1.0}
    iex> LineSegment.from_barycentric( segment, q)
    {1.0, 1.0, 1.0}

    iex> #IO.puts("Check coordinate at middle of line")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
    iex> q = {0.5, 0.5}
    iex> LineSegment.from_barycentric( segment, q)
    {0.0, 0.0, 0.0}

    iex> #IO.puts("Check coordinate in voronoi region of a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
    iex> q = {1.25, -0.25}
    iex> LineSegment.from_barycentric( segment, q)
    {-1.5, -1.5, -1.5}

    iex> #IO.puts("Check coordinate in voronoi region of b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
    iex> q = {-0.25, 1.25}
    iex> LineSegment.from_barycentric( segment, q)
    {1.5, 1.5, 1.5}
  """
  @spec from_barycentric( line_segment, {number, number}) :: Vec3.vec3
  def from_barycentric( line_segment(a: a, b: b), {u,v}) do
    Vec3.weighted_sum(u,a,v,b)
  end

  @doc """
  Projects a query point `q` onto the neartest point on line segment `ab`, giving the nearest point.

  ## Examples
    iex> #IO.puts("Check segment coincident with a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {0.0, 0.0, 0.0}
    iex> LineSegment.nearest_point( segment, q)
    {0.0, 0.0, 0.0}

    iex> #IO.puts("Check segment coincident with b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {1.0, 0.0, 0.0}
    iex> LineSegment.nearest_point( segment, q)
    {1.0, 0.0, 0.0}

    iex> #IO.puts("Check segment interior of segment")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {0.5, 0.0, 0.0}
    iex> LineSegment.nearest_point( segment, q)
    {0.5, 0.0, 0.0}

    iex> #IO.puts("Check segment in voronoi region of a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {-0.5, 0.0, 0.0}
    iex> LineSegment.nearest_point( segment, q)
    {0.0, 0.0, 0.0}

    iex> #IO.puts("Check segment in voronoi region of b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {1.5, 0.0, 0.0}
    iex> LineSegment.nearest_point( segment, q)
    {1.0, 0.0, 0.0}

    iex> #IO.puts("Check offset segment in voronoi region of a")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {-0.5, 3.0, 4.0}
    iex> LineSegment.nearest_point( segment, q)
    {0.0, 0.0, 0.0}

    iex> #IO.puts("Check offset segment in voronoi region of b")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {1.5, 3.0, 4.0}
    iex> LineSegment.nearest_point( segment, q)
    {1.0, 0.0, 0.0}

    iex> #IO.puts("Check offset segment in voronoi region of segment")
    iex> require ElixirRigidPhysics.Geometry.LineSegment, as: LineSegment
    iex> segment = {:line_segment, {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}
    iex> q = {0.45, 3.0, 4.0}
    iex> LineSegment.nearest_point( segment, q)
    {0.45, 0.0, 0.0}
  """
  @spec nearest_point( line_segment, Vec3.vec3) :: Vec3.vec3
  def nearest_point( line_segment(a: a, b: b), q) do
    segment_vec = Vec3.subtract(b,a)
    segment_vec_length = Vec3.length(segment_vec)
    segment_vec_unit = Vec3.scale(segment_vec, 1.0 / segment_vec_length)
    qa = Vec3.subtract(q,a)
    bq = Vec3.subtract(b,q)

    u = Vec3.dot(bq, segment_vec_unit) / segment_vec_length
    v = Vec3.dot(qa, segment_vec_unit) / segment_vec_length

    cond do
      v < 0 -> a
      u < 0 -> b
      true -> Vec3.weighted_sum(u,a,v,b)
    end
  end
end
