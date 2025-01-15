{
  description = "A very basic flake that generates the DBCs by running a python script";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }: flake-utils.lib.eachDefaultSystem (system: let
    pkgs = nixpkgs.legacyPackages.${system};

    pythonEnv = pkgs.python312.withPackages(ps: [
      ps.cantools
    ]);

  in {
    # We export this package so we can refrence its output in another flake as an overlay
    packages = {
      default = pkgs.callPackage ./default.nix {};
    };

    # This is so that the github build action can just use the nix script to ensure reproducablility and making my life easier lol
    apps.default = {
      type = "app";
      program = "${pythonEnv}/bin/python";
    };

    # I'm keeping the devshell for debug purposes
    devShells.default = pkgs.mkShell {
      name = "ksu_dbc";
      # Some deps needed to generate the output DBC
      buildInputs = [
        pkgs.python312
        pkgs.python312Packages.cantools
      ];
    };

    overlays.default = self: super: {
      default = super.callPackage ./default.nix {};
    };
  });
}
