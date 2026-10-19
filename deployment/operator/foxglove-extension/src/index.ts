import type { ExtensionContext } from '@foxglove/extension';

import { initImageGoalPanel } from './panel';

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({name:'ImageGoal',initPanel:initImageGoalPanel});
}
