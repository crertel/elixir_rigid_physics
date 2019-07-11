defmodule ElixirRigidPhysics.World do
  @moduledoc """
  The "world" of a ERP simulation.

  There can be zero or more worlds, and they are stepped explicitly.

  Worlds do not interact with one another.
  """

  @type t :: %__MODULE__{}

  defstruct process: nil,
            current_time: 0.0,
            timestep: 0,
            bodies: %{},
            last_step_usecs: 0,
            narrowphase_usecs: 0,
            broadphase_usecs: 0,
            acc_struct_pop_usecs: 0,
            broadphase_acceleration_structure:
              ElixirRigidPhysics.Collision.Broadphase.create_acceleration_structure(),
            collisions: []
end
