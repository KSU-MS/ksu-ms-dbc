# let
#  can_dbc_overlay = final: prev: {
#    can_pkg = final.callPackage ./default.nix { };
#  };
#  custom_overlays = [ can_dbc_overlay ];
#
#  pkgs = import nixpkgs {
#    system = "${system}";
#    overlays = [self.overlays.default];
#  };
#
# in {
#  # We export this package so that we can refrence this flake's output in the ksu_daq repo
#  overlays.default = nixpkgs.lib.composeManyExtensions custom_overlays;
#
#  packages.${system} = rec {
#    can_pkg = pkgs.can_pkg;
#    default = can_pkg;
#  };
# }

{
  description = "A very basic flake that generates the DBCs by running the python script";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
    flake-utils.url = "github:numtide/flake-utils";
  };
  outputs = { self, nixpkgs, flake-utils, ... }: flake-utils.lib.eachDefaultSystem (system: 
    let
      pkgs = nixpkgs.legacyPackages.${system};

      pythonEnv = pkgs.python312.withPackages(ps: [
        ps.cantools
      ]);

      # Import our nix script as an overlay
      # can_dbc_overlay = final: prev: {
      #   can_pkg = final.callPackage ./default.nix { };
      # };

    in {
      # We export this package so that we can refrence the output as an overlay in the ksu_daq repo
      # overlays.default = nixpkgs.lib.composeManyExtensions can_dbc_overlay;


      pkgs = import nixpkgs {
        overlays = [ 
          (self: super: {
            default = super.callPackage ./default.nix {};
          })
        ];
      };

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
