defmodule ElixirRigidPhysics.MixProject do
  use Mix.Project

  def project do
    [
      app: :elixir_rigid_physics,
      version: "0.1.0",
      elixir: "~> 1.8",
      start_permanent: Mix.env() == :prod,
      deps: deps(),
      docs: &docs/0,
      test_coverage: [tool: ExCoveralls],
      preferred_cli_env: [
        coveralls: :test,
        "coveralls.detail": :test,
        "coveralls.post": :test,
        "coveralls.html": :test
      ]
    ]
  end

  # Run "mix help compile.app" to learn about applications.
  def application do
    [
      extra_applications: [:logger]
    ]
  end

  # Run "mix help deps" to learn about dependencies.
  defp deps do
    [
      {:credo, "~> 1.5.6", only: :dev},
      {:dialyxir, "~> 1.1.0", only: [:dev], runtime: false},
      {:ex_doc, "~> 0.24.2", only: [:docs, :dev]},
      {:excoveralls, "~> 0.14.1", only: [:test, :dev]},
      {:inch_ex, "~> 2.0.0", only: :docs},
      {:graphmath, "~> 2.4.0"}
    ]
  end

  defp docs do
    {ref, 0} = System.cmd("git", ["rev-parse", "--verify", "--quiet", "HEAD"])
    [source_ref: ref, main: "api-reference"]
  end
end
