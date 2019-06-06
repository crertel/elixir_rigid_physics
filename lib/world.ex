defmodule ElixirRigidPhysics.World do
  @moduledoc """
  The "world" of a ERP simulation.

  There can be zero or more worlds, and they are stepped explicitly.

  Worlds do not interact with one another.
  """

  @type t :: %__MODULE__{}

  defstruct process: nil
end
