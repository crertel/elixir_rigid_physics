defmodule ElixirRigidPhysics.Util.List do

  @doc """
  Function to find every pair of elements in a list.

  ## Examples

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.generate_pairs([])
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.generate_pairs([1])
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.generate_pairs([1,2])
    [{1,2}]

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.generate_pairs([1,2,3])
    [{1,2}, {1,3}, {2,3}]
  """
  @spec generate_pairs( [any()] ) :: [ {any(), any()} ]
  def generate_pairs([]), do: []
  def generate_pairs([_]), do: []
  def generate_pairs([a,b]), do: [{a,b}]
  def generate_pairs([a | rest]) do
    main_pairs = for i <- rest, do: {a,i}
    main_pairs ++ generate_pairs(rest)
  end


  @doc """
  Function to find every pair of elements in a list.

  ## Examples

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.generate_pairs([], fn(_a,_b) ->  true end)
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.generate_pairs([1], fn(_a,_b) ->  true end)
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.generate_pairs([1,2], fn(_a,_b) ->  true end)
    [{1,2}]

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.generate_pairs([1,2], fn(_a,_b) -> false end)
    []

    iex> alias ElixirRigidPhysics.Util.List
    iex> List.generate_pairs([1,2,3], fn(a,b) -> a * b > 3 end)
    [{2,3}]
  """
  @spec generate_pairs( [any()], (any(), any() -> boolean()) ) :: [ {any(), any()} ]
  def generate_pairs([], _predicate), do: []
  def generate_pairs([_], _predicate), do: []
  def generate_pairs([a,b], predicate) do
    if (predicate.(a,b)) do
      [{a,b}]
    else
      []
    end
  end
  def generate_pairs([a | rest], predicate) do
    main_pairs = for i <- rest, predicate.(a,i), do: {a,i}
    main_pairs ++ generate_pairs(rest)
  end
end
