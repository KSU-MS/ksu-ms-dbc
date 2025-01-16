# I hate nix
{
  description = "A very basic flake that generates the DBCs by running a python script";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
  };

  outputs = { self, nixpkgs }: let
    pkg_overlay = final: prev: {
      can_pkg = final.callPackage ./default.nix { };
    };
    custom_overlays = [ pkg_overlay ];

    pkgs = import nixpkgs {
      system = "x86_64-linux";
      overlays = [ self.overlays.default ];
    };

  in {
    overlays.default = nixpkgs.lib.composeManyExtensions custom_overlays;

    packages.x86_64-linux = rec {
      can_pkg = pkgs.can_pkg;
      default = can_pkg;
    };
  };
}

# It makes me cry

# {
#   description = "A very basic flake that generates the DBCs by running a python script";
#   inputs = {
#     nixpkgs.url = "github:NixOS/nixpkgs/nixos-24.11";
#     flake-utils.url = "github:numtide/flake-utils";
#   };
#
#   outputs = { self, nixpkgs, flake-utils, ... }: flake-utils.lib.eachDefaultSystem (system: let
#     pkgs = nixpkgs.legacyPackages.${system};
#
#     pythonEnv = pkgs.python312.withPackages(ps: [
#       ps.cantools
#     ]);
#
#   in {
#     # We export this package so we can refrence its output in another flake as an overlay
#     packages = {
#       default = pkgs.callPackage ./default.nix {};
#     };
#
#     # This is so that the github build action can just use the nix script to ensure reproducablility and making my life easier lol
#     apps.default = {
#       type = "app";
#       program = "${pythonEnv}/bin/python";
#     };
#
#     # I'm keeping the devshell for debug purposes
#     devShells.default = pkgs.mkShell {
#       name = "ksu_dbc";
#       # Some deps needed to generate the output DBC
#       buildInputs = [
#         pkgs.python312
#         pkgs.python312Packages.cantools
#       ];
#
#       overlays.default = self: super: {
#         default = super.callPackage ./default.nix {};
#       };
#     };
#   });
# }
