{
  description = "A very basic flake that generates the DBCs by running the python script";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }: flake-utils.lib.eachDefaultSystem (system: 
    let
      pkgs = nixpkgs.legacyPackages.${system};
      pythonEnv = pkgs.python312.withPackages(ps: [
        ps.cantools
      ]);

      # Import our nix script as an overlay
      can_dbc_overlay = final: prev: {
        can_pkg = final.callPackage ./default.nix { };
      };

      # Unused bc flakes are stupid sometimes, here so I don't forget the command lol
      fella = "./json_to_dbc.py ./can_descriptor.json ./dbc-output/ksu_dbc Orion RMS_PM Megasquirt";

    in {
      # We export this overlay so that we can refrence this flake in the ksu_daq repo
      overlays.default = nixpkgs.lib.composeManyExtensions can_dbc_overlay;

      # This is so that the github build action can just use the nix script to ensure reproducablility and making my life easier lol
      defaultApp = {
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
    }
  );
}
