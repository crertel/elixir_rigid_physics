IO.puts "Setting up ERP workbench..."

# make inspect useful
IEx.configure(inspect: [limit: :infinity])

# alias our workhorse stuff
alias ElixirRigidPhysics, as: ERP
alias Graphmath.Vec2
alias Graphmath.Vec3
alias Graphmath.Quatern
alias ERP.Collision.Narrowphase
alias ERP.Collision.Intersection

# require our primitives
require ERP.Dynamics.Body, as: Body
require ERP.World, as: World
require ERP.Collision.AABB, as: AABB
require ERP.Collision.Broadphase, as: Broadphase
require ERP.Collision.Contact, as: Contact
require ERP.Geometry.Box, as: Box
require ERP.Geometry.Capsule, as: Capsule
require ERP.Geometry.Hull, as: Hull
require ERP.Geometry.LineSegment, as: LineSegment
require ERP.Geometry.Plane, as: Plane
require ERP.Geometry.Sphere, as: Sphere
require ERP.Geometry.Tetrahedron, as: Tetra
require ERP.Geometry.Triangle, as: Tri

# declare our reference tetrahedron qABCD
quad_vert_a = {0.0, 0.0, 0.0}
quad_vert_b = {0.0, 0.0, 1.0}
quad_vert_c = {1.0, 0.0, 0.0}
quad_vert_d = {0.0, 1.0, 0.0}
ref_tetra = Tetra.create(quad_vert_a, quad_vert_b, quad_vert_c, quad_vert_d)

IO.puts "Done setting up ERP workbench!"

