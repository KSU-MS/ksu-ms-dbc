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
